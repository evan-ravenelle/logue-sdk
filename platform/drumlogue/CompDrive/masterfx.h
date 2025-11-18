#pragma once
/*
 *  File: master.h
 *
 *  CompDrive Master Effect Class
 *
 *  Author: Evan Ravenelle
 *
 *  2023(c) Evan Ravenelle
 *
 */

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <arm_neon.h>

#include "unit.h"  // Note: Include common definitions for all units
#include "rms.h" // RMS calculator class
#include "envelope.h" // envelope detector

constexpr size_t c_parameterAttack = 0;
constexpr size_t c_parameterRelease = 1;
constexpr size_t c_parameterTHold= 2;
constexpr size_t c_parameterRatio = 3;
constexpr size_t c_parameterRMSBitmap = 4;
constexpr size_t c_parameterKnee = 5;
constexpr size_t c_parameterDrive = 6;
constexpr size_t c_parameterMakeup = 7;
constexpr size_t c_parameterDryWet = 8;
constexpr size_t c_parameterGain = 9;
constexpr size_t c_parameterSidechain = 10;
constexpr size_t c_parameterNoiseTHold = 12;
constexpr size_t c_parameterNoiseRelease = 13;
constexpr size_t c_parameterNoiseGateOn = 14;
constexpr size_t c_parameterNoiseTholdTime = 15;



class MasterFX {
 public:
  /*===========================================================================*/
  /* Public Data Structures/Types. */
  /*===========================================================================*/

  /*===========================================================================*/
  /* Lifecycle Methods. */
  /*===========================================================================*/

  MasterFX(void) :
    v_attack(30),
    v_release(300),
    v_threshold(0),
    v_ratio(1),
    v_drive(0),
    v_makeup(0),
    v_sampleRate(48000.0f),
    v_knee(1),
    v_dryWet(0),
    v_gain(0),
    v_sidechain(0),
    v_noiseThreshold(1),
    v_noiseRelease(0),
    v_noiseGateOn(0),
    unity(vdup_n_f32(1.0f)),
    envelope(vdup_n_f32(0.0f)),
    rms(vdupq_n_f32(0.0f)),
    noiseGain(vdup_n_f32(1.0f)),
    envDetector(48000.0f),
    RMS(2400, 48000.0f),  // Reduced window: 50ms at 48kHz for memory efficiency
    pregainCoeff(1.0f),
    driveLinear(1.0f),
    makeupCoeff(1.0f),
    noiseThresholdLinear(0.0f),
    noiseReleaseCoeff(1.0f)
  {
    // Initialize bitmap to all off
    for (int i = 0; i < 32; i++) {
      rmsBitmap[i] = 0x00U;
    }
  }

  virtual ~MasterFX(void) {}

  inline int8_t Init(const unit_runtime_desc_t * desc) {
    // Check compatibility of samplerate with unit, for drumlogue should be 48000
    if (desc->samplerate != 48000)
      return k_unit_err_samplerate;

    // Check compatibility of frame geometry
    // Note: input format: [ main_left, main_right, sidechain_left, sidechain_right ]
    if (desc->input_channels != 4 || desc->output_channels != 2)
      return k_unit_err_geometry;

    v_sampleRate = desc->samplerate;

    // Initialize envelope detector with correct sample rate
    envDetector = EnvelopeDetector(v_sampleRate);
    RMS = RMSCalculator(2400, v_sampleRate);  // 50ms window at 48kHz

    return k_unit_err_none;
  }

  inline void Teardown() {
    // Note: cleanup and release resources if any
  }

  inline void Reset() {
    // Reset effect state
    envelope = vdup_n_f32(0.0f);
    noiseGain = vdup_n_f32(1.0f);
    RMS.reset();
  }

  inline void Resume() {
    // Note: Effect will resume and exit suspend state. Usually means the synth
    // was selected and the render callback will be called again
  }

  inline void Suspend() {
    // Note: Effect will enter suspend state. Usually means another effect was
    // selected and thus the render callback will not be called
  }

  /*===========================================================================*/
  /* Other Public Methods. */
  /*===========================================================================*/

  // Fast inline helper: convert dB to linear amplitude using lookup or approximation
  inline float fastDbToLinear(float db) {
    // For real-time: powf(10.0f, db / 20.0f) is expensive
    // Using fast approximation or pre-calculated in setParameter is better
    return powf(10.0f, db / 20.0f);
  }

  inline float32x2_t toLinearAmplitude(float input) {
    return vdup_n_f32(fastDbToLinear(input));
  }

  inline float32x2_t sidechainedRMS(float32x2_t audioInput, float32x2_t sidechainInput) {
    uint32x2_t comparison = vcgt_f32(sidechainInput, audioInput);
    return vbsl_f32(comparison, sidechainInput, audioInput);
  }

  fast_inline void Process(const float * in, float * out, size_t frames) {
    // Declare all variables outside the loop
    float32x4_t inSamples;
    float32x2_t audioSamples;
    float32x2_t sidechainSamples;
    float32x2_t threshold;
    float32x2_t ratio;
    float32x2_t delta;
    uint32x2_t theta;
    float32x2_t compressionGain;
    float32x2_t kneeWidth;
    float32x2_t lowerThreshold;
    float32x2_t upperThreshold;
    uint32x2_t belowLower;
    uint32x2_t aboveUpper;
    float32x2_t slope;
    float32x2_t slopeDiv;
    float32x2_t kneeRatio;
    float32x2_t kneeGain;
    float32x2_t absAudioSamples;
    uint32x2_t gateCondition;
    float32x2_t gateGainTarget;
    float32x2_t dryWetParameter;
    float32x2_t inverseDryWet;
    float32x2_t drySamples;
    float32x2_t wetSamples;
    float32x2_t outputSamples;
    float32x2_t audioRMS;
    float32x2_t clampThreshold;
    uint32x2_t positiveClamp;
    float32x2_t clampedPositive;
    float32x2_t clampedNegative;

    const float * __restrict in_p = in;
    float * __restrict out_p = out;
    const float * out_e = out_p + (frames << 1);  // assuming stereo output

    for (; out_p != out_e; in_p += 4, out_p += 2) {
        // Load a vector of four input samples (main L/R, sidechain L/R)
        inSamples = vld1q_f32(in_p);

        // Extract audio and sidechain samples
        audioSamples = vget_low_f32(inSamples);
        sidechainSamples = vget_high_f32(inSamples);

        // Apply pre-gain (calculated in setParameter)
        audioSamples = vmul_f32(audioSamples, vdup_n_f32(pregainCoeff));

        // Update envelope follower using class member envDetector
        envelope = envDetector.processSample(inSamples, envelope, v_sidechain, v_attack, v_release);

        // Compression threshold and ratio (converted to linear in setParameter would be better)
        threshold = toLinearAmplitude(v_threshold);
        ratio = vdup_n_f32(1.0f / (v_ratio > 0 ? v_ratio : 1));

        // Calculate how far we are from threshold
        delta = vsub_f32(envelope, threshold);
        theta = vcgt_f32(delta, vdup_n_f32(0.0f)); // true means above threshold

        // Soft knee compression
        if (v_knee != 0) {
            kneeWidth = toLinearAmplitude(v_knee);
            // Using vdiv would be better but may not be available, use reciprocal estimate
            float32x2_t quarter = vdup_n_f32(0.25f);
            lowerThreshold = vsub_f32(threshold, vmul_f32(kneeWidth, quarter));
            upperThreshold = vadd_f32(threshold, vmul_f32(kneeWidth, quarter));

            belowLower = vclt_f32(envelope, lowerThreshold);
            aboveUpper = vcgt_f32(envelope, upperThreshold);

            // Calculate soft knee slope
            // slope = (1 - ratio) / kneeWidth_dB
            float kneeWidthDb = v_knee / 2.0f;
            float slopeScalar = (1.0f - (1.0f / (v_ratio > 0 ? v_ratio : 1))) / (kneeWidthDb > 0 ? kneeWidthDb : 1);

            // Knee gain calculation: simplified approach
            float32x2_t excessDb = vmul_f32(delta, vdup_n_f32(slopeScalar));
            kneeGain = vsub_f32(unity, excessDb);

            // Select appropriate gain: unity below knee, kneeGain in knee, ratio above
            compressionGain = vbsl_f32(belowLower, unity, kneeGain);
            compressionGain = vbsl_f32(aboveUpper, ratio, compressionGain);
        } else {
            // Hard knee: apply ratio above threshold, unity below
            compressionGain = vbsl_f32(theta, ratio, unity);
        }

        audioSamples = vmul_f32(audioSamples, compressionGain);

        // Soft clipping / drive (calculated in setParameter)
        if (v_drive > 0) {
            clampThreshold = vdup_n_f32(driveLinear);
            positiveClamp = vcge_f32(audioSamples, vdup_n_f32(0.0f));

            // Positive samples: clamp to +driveLinear
            clampedPositive = vmin_f32(audioSamples, clampThreshold);
            // Negative samples: clamp to -driveLinear
            clampedNegative = vmax_f32(audioSamples, vneg_f32(clampThreshold));

            audioSamples = vbsl_f32(positiveClamp, clampedPositive, clampedNegative);
        }

        // Noise Gate (using pre-calculated coefficients)
        if (v_noiseGateOn) {
            absAudioSamples = vabs_f32(audioSamples);
            gateCondition = vclt_f32(absAudioSamples, vdup_n_f32(noiseThresholdLinear));

            // If below threshold, reduce gain; otherwise target gain = 1.0
            gateGainTarget = vbsl_f32(gateCondition, vdup_n_f32(0.0f), unity);

            // Smooth the gain change
            float32x2_t gainDelta = vsub_f32(gateGainTarget, noiseGain);
            float32x2_t coeffToUse = vbsl_f32(gateCondition,
                                              vdup_n_f32(noiseReleaseCoeff),
                                              vdup_n_f32(0.1f)); // Fast attack for gate open
            noiseGain = vadd_f32(noiseGain, vmul_f32(gainDelta, vsub_f32(unity, coeffToUse)));

            audioSamples = vmul_f32(audioSamples, noiseGain);
        }

        // Apply makeup gain (calculated in setParameter)
        audioSamples = vmul_f32(audioSamples, vdup_n_f32(makeupCoeff));

        // Blend Dry/Wet
        float dryWetNorm = (v_dryWet + 100.0f) / 200.0f;
        dryWetParameter = vdup_n_f32(dryWetNorm);
        inverseDryWet = vsub_f32(unity, dryWetParameter);

        drySamples = vmul_f32(vget_low_f32(inSamples), inverseDryWet);
        wetSamples = vmul_f32(audioSamples, dryWetParameter);
        outputSamples = vadd_f32(drySamples, wetSamples);

        // Update RMS display
        RMS.addValue(vabsq_f32(inSamples));

        // Store output
        vst1_f32(out_p, outputSamples);
    }

    // Update RMS bitmap once per buffer (not every sample!)
    rms = RMS.getRMS(v_attack, v_release);
    audioRMS = vget_low_f32(rms);
    updateBitmapWithRMS(audioRMS, rmsBitmap);
  }

  const float dBThresholds[16] = {
    -60.0f, -40.0f, -30.0f, -20.0f, -15.0f, -12.0f, -9.0f, -6.0f,
    -4.5f, -3.0f, 0.0f, -5.0f, 0.0f, -1000.0, 3.0f, 6.0f
};

   const float linearAmplitudes[16] = {
    std::pow(10.0f, dBThresholds[0] / 20.0f),  
    std::pow(10.0f, dBThresholds[1] / 20.0f),  
    std::pow(10.0f, dBThresholds[2] / 20.0f),  
    std::pow(10.0f, dBThresholds[3] / 20.0f),  
    std::pow(10.0f, dBThresholds[4] / 20.0f),  
    std::pow(10.0f, dBThresholds[5] / 20.0f),  
    std::pow(10.0f, dBThresholds[6] / 20.0f),  
    std::pow(10.0f, dBThresholds[7] / 20.0f),  
    std::pow(10.0f, dBThresholds[8] / 20.0f),  
    std::pow(10.0f, dBThresholds[9] / 20.0f),  
    std::pow(10.0f, dBThresholds[10] / 20.0f), 
    std::pow(10.0f, dBThresholds[11] / 20.0f), 
    std::pow(10.0f, dBThresholds[12] / 20.0f), 
    std::pow(10.0f, dBThresholds[13] / 20.0f), 
    std::pow(10.0f, dBThresholds[14] / 20.0f), 
    std::pow(10.0f, dBThresholds[15] / 20.0f)  
};

inline void updateBitmapWithRMS(float32x2_t rms, uint8_t* bitmap) {
    uint32x2_t rowMask;
    uint32x2_t rowResult;

    for (int row = 0; row < 16; row += 2) {
        rowMask = vcge_f32(rms, vdup_n_f32(linearAmplitudes[row]));
        rowResult = vbsl_u32(rowMask, vdup_n_u32(0xFFU), vdup_n_u32(0x00U));

        bitmap[row] = vget_lane_u32(rowResult, 0) & 0xFF;
        bitmap[row + 1] = vget_lane_u32(rowResult, 1) & 0xFF;
    }
  }


  inline void setParameter(uint8_t index, int32_t value) {
    switch (index) {
        case c_parameterAttack:
            v_attack = value;
            break;

        case c_parameterRelease:
            v_release = value;
            break;

        case c_parameterTHold:
            v_threshold = value;
            break;

        case c_parameterRatio:
            v_ratio = value;
            break;

        case c_parameterKnee:
            v_knee = value;
            break;

        case c_parameterDrive:
            v_drive = value;
            // Pre-calculate linear drive value for clipping
            driveLinear = powf(10.0f, -value / 20.0f);
            break;

        case c_parameterMakeup:
            v_makeup = value;
            // Pre-calculate makeup gain coefficient
            makeupCoeff = 1.0f + powf(10.0f, value / 20.0f);
            break;

        case c_parameterDryWet:
            v_dryWet = value;
            break;

        case c_parameterGain:
            v_gain = value;
            // Pre-calculate pre-gain coefficient
            pregainCoeff = 1.0f + powf(10.0f, value / 20.0f);
            break;

        case c_parameterSidechain:
            v_sidechain = value;
            break;

        case c_parameterNoiseTHold:
            v_noiseThreshold = value;
            // Pre-calculate noise threshold in linear
            noiseThresholdLinear = powf(10.0f, value / 20.0f);
            break;

        case c_parameterNoiseRelease:
            v_noiseRelease = value;
            // Pre-calculate noise gate release coefficient
            if (v_sampleRate > 0 && value > 0) {
                noiseReleaseCoeff = expf(-1.0f / ((value / 1000.0f) * v_sampleRate));
            } else {
                noiseReleaseCoeff = 0.999f;
            }
            break;

        case c_parameterNoiseGateOn:
            v_noiseGateOn = value;
            break;

        default:
            break;
    }
  }

  inline int32_t getParameterValue(uint8_t index) const {
    switch (index) {
        case c_parameterAttack:
            return v_attack;
        case c_parameterRelease:
            return v_release;
        case c_parameterTHold:
            return v_threshold;
        case c_parameterRatio:
            return v_ratio;
        case c_parameterKnee:
            return v_knee;
        case c_parameterDrive:
            return v_drive;
        case c_parameterMakeup:
            return v_makeup;
        case c_parameterDryWet:
            return v_dryWet;
        case c_parameterGain:
            return v_gain;
        case c_parameterSidechain:
            return v_sidechain;
        case c_parameterNoiseTHold:
            return v_noiseThreshold;
        case c_parameterNoiseRelease:
            return v_noiseRelease; 
        case c_parameterNoiseGateOn:
            return  v_noiseGateOn;
        default:
            return 0; 
    }
    return 0;
  }

  inline const char * getParameterStrValue(uint8_t index, int32_t value) const {
    (void)value;
    switch (index) {
      // Note: String memory must be accessible even after function returned.
      //       It can be assumed that caller will have copied or used the string
      //       before the next call to getParameterStrValue
      default:
        break;
    }
    return nullptr;
  }

  inline const uint8_t * getParameterBmpValue(uint8_t index,
                                              int32_t value) const {
    (void)value;
    switch (index) {
        case c_parameterRMSBitmap:
            return rmsBitmap;

      // Note: Bitmap memory must be accessible even after function returned.
      //       It can be assumed that caller will have copied or used the bitmap
      //       before the next call to getParameterBmpValue
      // Note: Not yet implemented upstream

      default:
        break;
    }
    return nullptr;
  }

  inline void LoadPreset(uint8_t idx) { (void)idx; }

  inline uint8_t getPresetIndex() const { return 0; }

  /*===========================================================================*/
  /* Static Members. */
  /*===========================================================================*/

  static inline const char * getPresetName(uint8_t idx) {
    (void)idx;
    // Note: String memory must be accessible even after function returned.
    //       It can be assumed that caller will have copied or used the string
    //       before the next call to getPresetName
    return nullptr;
  }

 private:
  /*===========================================================================*/
  /* Private Member Variables. */
  /*===========================================================================*/

  // Parameter values
  uint32_t v_attack;
  uint32_t v_release;
  int8_t v_threshold;
  uint8_t v_ratio;
  uint16_t v_drive;
  uint16_t v_makeup;
  float v_sampleRate;
  int8_t v_knee;
  int8_t v_dryWet;
  int8_t v_gain;
  uint8_t v_sidechain;
  int8_t v_noiseThreshold;
  float v_noiseRelease;
  uint8_t v_noiseGateOn;

  // NEON constants and state
  float32x2_t unity;
  float32x2_t envelope;
  float32x4_t rms;
  float32x2_t noiseGain;

  // DSP objects
  EnvelopeDetector envDetector;
  RMSCalculator RMS;

  // Pre-calculated coefficients (to avoid expensive operations in audio loop)
  float pregainCoeff;
  float driveLinear;
  float makeupCoeff;
  float noiseThresholdLinear;
  float noiseReleaseCoeff;

  // Display bitmap
  uint8_t rmsBitmap[32];

  /*===========================================================================*/
  /* Private Methods. */
  /*===========================================================================*/

  /*===========================================================================*/
  /* Constants. */
  /*===========================================================================*/
};

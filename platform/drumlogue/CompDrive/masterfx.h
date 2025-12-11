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
    noiseReleaseCoeff(0.99f),
    noiseAttackCoeff(0.1f),
    thresholdLinear(1.0f),
    ratioLinear(1.0f),
    kneeWidthLinear(1.0f),
    kneeSlopeScalar(0.0f),
    dryWetCoeff(0.5f),
    attackCoeff(0.99f),
    releaseCoeff(0.99f)
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

  fast_inline void Process(const float * in, float * out, size_t frames) {
    // Declare loop variables
    float32x4_t inSamples;
    float32x2_t audioSamples;
    float32x2_t sidechainSamples;
    float32x2_t delta;
    uint32x2_t theta;
    float32x2_t compressionGain;
    float32x2_t lowerThreshold;
    float32x2_t upperThreshold;
    uint32x2_t belowLower;
    uint32x2_t aboveUpper;
    float32x2_t kneeGain;
    float32x2_t absAudioSamples;
    uint32x2_t gateCondition;
    float32x2_t gateGainTarget;
    float32x2_t drySamples;
    float32x2_t wetSamples;
    float32x2_t outputSamples;
    float32x2_t audioRMS;
    uint32x2_t positiveClamp;
    float32x2_t clampedPositive;
    float32x2_t clampedNegative;
    float32x2_t envInput;
    uint32x2_t envRising;
    float32x2_t envCoeff;
    float32x2_t envDelta;
    uint32x2_t sidechainMask;

    const float * __restrict in_p = in;
    float * __restrict out_p = out;
    const float * out_e = out_p + (frames << 1);  // assuming stereo output

    // Pre-load all SIMD constants (avoids repeated vdup_n_f32 in loop)
    const float32x2_t zeroVec = vdup_n_f32(0.0f);
    const float32x2_t attackCoeffVec = vdup_n_f32(attackCoeff);
    const float32x2_t releaseCoeffVec = vdup_n_f32(releaseCoeff);
    const float32x2_t thresholdVec = vdup_n_f32(thresholdLinear);
    const float32x2_t ratioVec = vdup_n_f32(ratioLinear);
    const float32x2_t pregainVec = vdup_n_f32(pregainCoeff);
    const float32x2_t makeupVec = vdup_n_f32(makeupCoeff);
    const float32x2_t dryWetVec = vdup_n_f32(dryWetCoeff);
    const float32x2_t inverseDryWetVec = vsub_f32(unity, dryWetVec);
    const float32x2_t denormalOffset = vdup_n_f32(DENORMAL_OFFSET);
    const float32x2_t kneeWidthVec = vdup_n_f32(kneeWidthLinear);
    const float32x2_t quarterVec = vdup_n_f32(0.25f);
    const float32x2_t kneeSlopeVec = vdup_n_f32(kneeSlopeScalar);
    const float32x2_t driveThresholdVec = vdup_n_f32(driveLinear);
    const float32x2_t noiseThresholdVec = vdup_n_f32(noiseThresholdLinear);
    const float32x2_t noiseAttackCoeffVec = vdup_n_f32(noiseAttackCoeff);
    const float32x2_t noiseReleaseCoeffVec = vdup_n_f32(noiseReleaseCoeff);

    for (; out_p != out_e; in_p += 4, out_p += 2) {
        // Load a vector of four input samples (main L/R, sidechain L/R)
        inSamples = vld1q_f32(in_p);

        // Extract audio and sidechain samples
        audioSamples = vget_low_f32(inSamples);
        sidechainSamples = vget_high_f32(inSamples);

        // Apply pre-gain
        audioSamples = vmul_f32(audioSamples, pregainVec);

        // Sidechain selection using SIMD (avoids branch)
        sidechainMask = vceq_u32(vdup_n_u32(v_sidechain), vdup_n_u32(1));
        float32x2_t absAudio = vabs_f32(audioSamples);
        float32x2_t absSidechain = vabs_f32(sidechainSamples);
        envInput = vbsl_f32(sidechainMask, absSidechain, absAudio);

        // Smooth envelope with attack/release
        envRising = vcgt_f32(envInput, envelope);
        envCoeff = vbsl_f32(envRising, attackCoeffVec, releaseCoeffVec);
        envDelta = vsub_f32(envInput, envelope);
        envelope = vadd_f32(envelope, vmul_f32(envDelta, vsub_f32(unity, envCoeff)));

        // Denormal prevention
        envelope = vadd_f32(envelope, denormalOffset);

        // Compression: calculate distance from threshold
        delta = vsub_f32(envelope, thresholdVec);
        theta = vcgt_f32(delta, zeroVec);

        // Soft knee compression
        if (v_knee != 0) {
            lowerThreshold = vsub_f32(thresholdVec, vmul_f32(kneeWidthVec, quarterVec));
            upperThreshold = vadd_f32(thresholdVec, vmul_f32(kneeWidthVec, quarterVec));

            belowLower = vclt_f32(envelope, lowerThreshold);
            aboveUpper = vcgt_f32(envelope, upperThreshold);

            // Knee gain using pre-calculated slope
            kneeGain = vsub_f32(unity, vmul_f32(delta, kneeSlopeVec));

            // Select appropriate gain: unity below knee, kneeGain in knee, ratio above
            compressionGain = vbsl_f32(belowLower, unity, kneeGain);
            compressionGain = vbsl_f32(aboveUpper, ratioVec, compressionGain);
        } else {
            // Hard knee: apply ratio above threshold, unity below
            compressionGain = vbsl_f32(theta, ratioVec, unity);
        }

        audioSamples = vmul_f32(audioSamples, compressionGain);

        // Soft clipping / drive
        if (v_drive > 0) {
            positiveClamp = vcge_f32(audioSamples, zeroVec);

            // Positive samples: clamp to +driveLinear
            clampedPositive = vmin_f32(audioSamples, driveThresholdVec);
            // Negative samples: clamp to -driveLinear
            clampedNegative = vmax_f32(audioSamples, vneg_f32(driveThresholdVec));

            audioSamples = vbsl_f32(positiveClamp, clampedPositive, clampedNegative);
        }

        // Smooth Noise Gate with EMA (prevents clicks and CPU halt from denormals)
        if (v_noiseGateOn) {
            absAudioSamples = vabs_f32(audioSamples);
            gateCondition = vclt_f32(absAudioSamples, noiseThresholdVec);

            // Target: below threshold = 0.0, above = 1.0
            gateGainTarget = vbsl_f32(gateCondition, zeroVec, unity);

            // Smooth EMA: use attack coeff when opening, release when closing
            float32x2_t gainDelta = vsub_f32(gateGainTarget, noiseGain);
            uint32x2_t gateRising = vcgt_f32(gainDelta, zeroVec);
            float32x2_t gateCoeffToUse = vbsl_f32(gateRising,
                                                  noiseAttackCoeffVec,
                                                  noiseReleaseCoeffVec);

            // Apply smooth EMA
            noiseGain = vadd_f32(noiseGain, vmul_f32(gainDelta, vsub_f32(unity, gateCoeffToUse)));

            // Denormal prevention for gate gain
            noiseGain = vadd_f32(noiseGain, denormalOffset);

            audioSamples = vmul_f32(audioSamples, noiseGain);
        }

        // Apply makeup gain
        audioSamples = vmul_f32(audioSamples, makeupVec);

        // Blend Dry/Wet
        drySamples = vmul_f32(vget_low_f32(inSamples), inverseDryWetVec);
        wetSamples = vmul_f32(audioSamples, dryWetVec);
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

  static constexpr float dBThresholds[16] = {
    -60.0f, -40.0f, -30.0f, -20.0f, -15.0f, -12.0f, -9.0f, -6.0f,
    -4.5f, -3.0f, 0.0f, -5.0f, 0.0f, -1000.0, 3.0f, 6.0f
};

   static constexpr float linearAmplitudes[16] = {
    0.001f,     // -60 dB
    0.01f,      // -40 dB
    0.03162f,   // -30 dB
    0.1f,       // -20 dB
    0.1778f,    // -15 dB
    0.2512f,    // -12 dB
    0.3548f,    // -9 dB
    0.5012f,    // -6 dB
    0.5957f,    // -4.5 dB
    0.7079f,    // -3 dB
    1.0f,       // 0 dB
    0.5623f,    // -5 dB
    1.0f,       // 0 dB
    0.0f,       // -1000 dB (essentially 0)
    1.4125f,    // +3 dB
    1.9953f     // +6 dB
};

inline void updateBitmapWithRMS(float32x2_t rms, uint8_t* bitmap) {
    // Take the maximum of left and right channels for the meter display
    float maxRMS = vget_lane_f32(rms, 0);
    float rightRMS = vget_lane_f32(rms, 1);
    if (rightRMS > maxRMS) {
        maxRMS = rightRMS;
    }

    // Fill bitmap based on amplitude thresholds
    // Each row lights up if signal is above that threshold
    for (int row = 0; row < 16; row++) {
        if (maxRMS >= linearAmplitudes[row]) {
            bitmap[row] = 0xFFU;  // Light up this row
        } else {
            bitmap[row] = 0x00U;  // Turn off this row
        }
    }

    // Fill the remaining bitmap bytes (32 bytes total, only using first 16)
    for (int row = 16; row < 32; row++) {
        bitmap[row] = 0x00U;
    }
  }


  inline void setParameter(uint8_t index, int32_t value) {
    switch (index) {
        case c_parameterAttack:
            v_attack = value;
            // Pre-calculate attack coefficient for envelope detector
            if (v_sampleRate > 0 && value > 0) {
                attackCoeff = expf(-1.0f / ((value / 1000.0f) * v_sampleRate + 1e-6f));
            } else {
                attackCoeff = 0.99f;
            }
            break;

        case c_parameterRelease:
            v_release = value;
            // Pre-calculate release coefficient for envelope detector
            if (v_sampleRate > 0 && value > 0) {
                releaseCoeff = expf(-1.0f / ((value / 1000.0f) * v_sampleRate + 1e-6f));
            } else {
                releaseCoeff = 0.99f;
            }
            break;

        case c_parameterTHold:
            v_threshold = value;
            // Pre-calculate threshold in linear domain
            thresholdLinear = powf(10.0f, value / 20.0f);
            break;

        case c_parameterRatio:
            v_ratio = value;
            // Pre-calculate ratio in linear domain
            ratioLinear = 1.0f / (value > 0 ? value : 1.0f);
            // Update knee slope (depends on ratio)
            if (v_knee != 0) {
                float kneeWidthDb = v_knee / 2.0f;
                kneeSlopeScalar = (1.0f - ratioLinear) / (kneeWidthDb > 0 ? kneeWidthDb : 1.0f);
            }
            break;

        case c_parameterKnee:
            v_knee = value;
            // Pre-calculate knee width in linear domain
            if (value != 0) {
                kneeWidthLinear = powf(10.0f, value / 20.0f);
                // Pre-calculate knee slope
                float kneeWidthDb = value / 2.0f;
                kneeSlopeScalar = (1.0f - ratioLinear) / (kneeWidthDb > 0 ? kneeWidthDb : 1.0f);
            } else {
                kneeWidthLinear = 0.0f;
                kneeSlopeScalar = 0.0f;
            }
            break;

        case c_parameterDrive:
            v_drive = value;
            // Pre-calculate linear drive value for clipping (fixed: removed negative sign)
            driveLinear = powf(10.0f, value / 20.0f);
            break;

        case c_parameterMakeup:
            v_makeup = value;
            // Pre-calculate makeup gain coefficient (fixed: removed + 1.0f)
            makeupCoeff = powf(10.0f, value / 20.0f);
            break;

        case c_parameterDryWet:
            v_dryWet = value;
            // Pre-calculate dry/wet coefficient
            dryWetCoeff = (value + 100.0f) / 200.0f;
            break;

        case c_parameterGain:
            v_gain = value;
            // Pre-calculate pre-gain coefficient (fixed: removed + 1.0f)
            pregainCoeff = powf(10.0f, value / 20.0f);
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
            // Pre-calculate noise gate release coefficient with slower, smoother EMA
            if (v_sampleRate > 0 && value > 0) {
                noiseReleaseCoeff = expf(-1.0f / ((value / 1000.0f) * v_sampleRate));
            } else {
                noiseReleaseCoeff = 0.99f;
            }
            // Attack is much faster but still smooth
            noiseAttackCoeff = 0.1f;  // Very fast attack
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
  float noiseAttackCoeff;

  // Pre-calculated compression parameters
  float thresholdLinear;
  float ratioLinear;
  float kneeWidthLinear;
  float kneeSlopeScalar;  // Pre-calculated knee slope for soft-knee compression
  float dryWetCoeff;

  // Attack/release coefficients for envelope detector
  float attackCoeff;
  float releaseCoeff;

  // Display bitmap
  uint8_t rmsBitmap[32];

  // Denormal prevention constant
  static constexpr float DENORMAL_OFFSET = 1e-20f;

  /*===========================================================================*/
  /* Private Methods. */
  /*===========================================================================*/

  /*===========================================================================*/
  /* Constants. */
  /*===========================================================================*/
};

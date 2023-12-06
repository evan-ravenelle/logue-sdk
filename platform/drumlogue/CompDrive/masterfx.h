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


 uint32_t v_attack;
 uint32_t v_release;
 uint8_t v_threshold;
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


 
class MasterFX {
 public:
  /*===========================================================================*/
  /* Public Data Structures/Types. */
  /*===========================================================================*/

  /*===========================================================================*/
  /* Lifecycle Methods. */
  /*===========================================================================*/

  MasterFX(void) {}
  virtual ~MasterFX(void) {}

  inline int8_t Init(const unit_runtime_desc_t * desc) {

    
    
    v_attack = 30;
    v_release = 300;
    v_threshold = 0;
    v_ratio = 1;
    v_drive = 0;
    v_makeup = 0;
    v_sampleRate = 48000.0f;
    v_knee = 1;
    v_dryWet = 0;
    v_gain = 0;
    v_noiseThreshold = 1;
    v_noiseRelease = 0;
    v_noiseGateOn = 0;
    float rmsSamples = 12000.0f;
    float32x2_t envelope = vdup_n_f32(0.0f);

    uint8_t rmsBitmap[32] = {
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU,
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU, 
      0xFFU, 0xFFU };
    
   


  // Check compatibility of samplerate with unit, for drumlogue should be 48000
    if (desc->samplerate != 48000)  // Note: samplerate format may change to add fractional bits
      return k_unit_err_samplerate;

    // Check compatibility of frame geometry
    // Note: input format: [ main_left, main_right, sidechain_left, sidechain_right ]
    // Note: Sidechain feature may unfortunately get removed before official release, in which case
    //       there will be only 2 input channels
    if (desc->input_channels != 4 || desc->output_channels != 2)
      return k_unit_err_geometry;

    // Note: if need to allocate some memory can do it here and return k_unit_err_memory if getting allocation errors

    return k_unit_err_none;
  }

  inline void Teardown() {
    // Note: cleanup and release resources if any
  }

  inline void Reset() {
    // Note: Reset effect state.
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

    inline float32x2_t toDecibel(float input) {
        return vdup_n_f32(20 * log10f(fabs(input)));
    };
    
    inline float32x2_t toDecibel32x2(float32x2_t input) {
        float32x2_t result;
        float32x2_t absInput = vabs_f32(input);
    
        for (int i = 0; i < 2; i++) {
            float value = vget_lane_f32(absInput, i);
            float decibel = 20.0f * log10f(value);
            vset_lane_f32(decibel, result, i);
        }
    
        return result;
    };

   inline float32x2_t toLinearAmplitude(float input) {
       return vdup_n_f32(pow(10.0f, input / 20.0f));
   }

   inline float32x2_t sidechainedRMS(float32x2_t audioInput, float32x2_t sidechainInput) {
       uint32x2_t comparison = vcgt_f32(sidechainInput, audioInput);
       return vbsl_f32(comparison, sidechainInput, audioInput);
   }

  fast_inline void Process(const float * in, float * out, size_t frames) {
    EnvelopeDetector envDetector = EnvelopeDetector(v_sampleRate);
    float32x4_t inSamples;  
    float32x2_t audioSamples;
    float32x2_t sidechainSamples;
    float32x2_t pregain;
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
    float32x2_t kneeRatio;
    float linear_drive;
    float32x2_t noiseGain;
    float32x2_t noise_threshold;
    float32x2_t noiseRelCoeff;
    float32x2_t absAudioSamples;
    uint32x2_t condition;
    float32x2_t postgain;
    float _dryWetParameter;
    float32x2_t dryWetParameter;
    float32x2_t inverseDryWet;
    float32x2_t dryMixingFactor;
    float32x2_t wetMixingFactor;
    float32x2_t drySamples;
    float32x2_t wetSamples;
    float32x2_t outputSamples;
    float32x2_t audioRMS;
    
    
    const float * __restrict in_p = in;
    float * __restrict out_p = out;
    const float * out_e = out_p + (frames << 1);  // assuming stereo output
   
        for (; out_p != out_e; in_p += 4, out_p += 2) {
    
        // Load a vector of four input samples (two stereo audio pairs and two sidechain pairs)
        inSamples = vld1q_f32(in_p);

        // Extract audio and sidechain samples using ARM NEON intrinsics
        audioSamples = vget_low_f32(inSamples);
        sidechainSamples =vget_high_f32(inSamples);

        // Apply pre-gain using ARM NEON intrinsics
        pregain = vdup_n_f32(1.0f + powf(10.0f, v_gain / 20.0f));
        audioSamples = vmul_f32(audioSamples, pregain);

        
        envelope = envDetector.processSample(inSamples, envelope, v_sidechain, v_attack, v_release);

        threshold = toLinearAmplitude(v_threshold);
        ratio = vdup_n_f32(1.0f/v_ratio);

        delta = vsub_f32(threshold, envelope); // delta of RMS/envelope and compression threshold
        theta = vcle_f32(delta, vdup_n_f32(0.0f)); // true means above compression threshold
        
        if (v_knee != 0) {
        
            kneeWidth = toLinearAmplitude(v_knee);
            lowerThreshold = vsub_f32(threshold, (kneeWidth/ vdup_n_f32(4.0f))); // div by 4 because knee is 0-12db stored as 0-24 for half values
            upperThreshold = vadd_f32(threshold, (kneeWidth / vdup_n_f32(4.0f)));
            belowLower = vclt_f32(delta, lowerThreshold);
            aboveUpper = vcgt_f32(delta, upperThreshold);
            
            slope = vsub_f32(ratio, vdup_n_f32(1.0f)) / vdup_n_f32(v_knee / 2.0f); //slope is based on knee in dB
            kneeRatio = vadd_f32(vdup_n_f32(1.0f), vmul_f32(vneg_f32(delta), slope));

            kneeRatio = vmul_f32(vmul_f32(vdup_n_f32(-1.0f),delta), vsub_f32(vdup_n_f32(1.0f), (vdup_n_f32(1.0) / kneeRatio)));
            compressionGain = vbsl_f32(belowLower, unity, kneeRatio);
            compressionGain = vbsl_f32(aboveUpper, ratio, compressionGain);
        } else {
            compressionGain = vbsl_f32(theta, ratio, unity);
        }
        
        audioSamples = vmul_f32(audioSamples, compressionGain);   
        

       if (v_drive > 0) {
            linear_drive = pow(10.0f, -v_drive / 20.0f);
            audioSamples = vcge_f32(audioSamples, vdup_n_f32(0.0f)) ? vmin_f32(audioSamples, vdup_n_f32(linear_drive)) : vmax_f32(audioSamples, vdup_n_f32(-linear_drive));
        }

        // Noise Gate

       if (v_noiseGateOn) {
            noiseGain = vdup_n_f32(1.0f);

            // Calculate noise_threshold using NEON intrinsics
            noise_threshold = vdup_n_f32(powf(10.0f, v_noiseThreshold / 20.0f));

            // Calculate release_coeff using NEON intrinsics
            noiseRelCoeff = vdup_n_f32(exp(-1.0f / ((v_noiseRelease / 1000.0f) * v_sampleRate)));
            // Apply the noise gate and gain adjustment
            absAudioSamples = vabs_f32(audioSamples);
            condition = vclt_f32(absAudioSamples, noise_threshold);
            noiseGain = vmin_f32(noiseGain, vmul_f32(noiseGain, noiseRelCoeff));
            noiseGain = vadd_f32(noiseGain, vmul_f32(noiseGain, vreinterpret_f32_u32(condition)));
           
            audioSamples = vmul_f32(audioSamples, noiseGain);
        }

        // Apply makeup gain using ARM NEON intrinsics
        postgain = vdup_n_f32(1.0f + powf(10.0f, v_makeup / 20.0f));
        audioSamples = vmul_f32(audioSamples, postgain);

        // Blend Dry/Wet
        _dryWetParameter = (v_dryWet + 100.0f) / 200.0f;

        dryWetParameter = vdup_n_f32(_dryWetParameter);

        inverseDryWet = vsub_f32(vdup_n_f32(1.0f), dryWetParameter);

        dryMixingFactor = vmul_f32(inverseDryWet, vdup_n_f32(0.5f));

        wetMixingFactor = vmul_f32(dryWetParameter, vdup_n_f32(0.5f));

        drySamples = vmul_f32(vget_low_f32(inSamples), dryMixingFactor);

        wetSamples = vmul_f32(audioSamples, wetMixingFactor);

        outputSamples = vadd_f32(drySamples, wetSamples);

        RMS.addValue(vabsq_f32(inSamples));
        rms = RMS.getRMS(v_attack, v_release);
        audioRMS = vget_low_f32(rms);
        updateBitmapWithRMS(audioRMS,rmsBitmap);

        vst1_f32(out_p, outputSamples);
    }
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

inline uint8_t updateBitmapWithRMS(float32x2_t rms, uint8_t* bitmap) {
    uint32x2_t rowMask;
    uint32x2_t rowResult;
    
        for (int row = 0; row < 16; row+=2) {
            rowMask = vcge_f32(rms, vdup_n_f32(linearAmplitudes[row]));
            rowResult = vbsl_u32(rowMask, vdup_n_u32(1), vdup_n_u32(0) );

            bitmap[row] =  vget_lane_u32(rowResult, 0);
            bitmap[row + 1] = vget_lane_u32(rowResult, 1);
        }
    
    
    }


  inline void setParameter(uint8_t index, int32_t value) {
    (void)value;
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
            break;

        case c_parameterMakeup:
            v_makeup = value;
            break;
        case c_parameterDryWet:
            v_dryWet = value;
            break;
        case c_parameterGain:
            v_gain = value;
            break;
        case c_parameterSidechain:
            v_sidechain = value;
            break;
         case c_parameterNoiseTHold:
            v_noiseThreshold = value;
            break;
        case c_parameterNoiseRelease:
            v_noiseRelease = value; 
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

  std::atomic_uint_fast32_t flags_;
  float32x2_t unity = vdup_n_f32(1.0f);
  float32x2_t envelope;
  float32x4_t rms;
  float rmsSamples;
  float32x2_t noiseSmoothing = vdup_n_f32(1.0f);
  RMSCalculator RMS = RMSCalculator(rmsSamples, v_sampleRate);
  uint8_t rmsBitmap[32]; 
  
 

  /*===========================================================================*/
  /* Private Methods. */
  /*===========================================================================*/

  /*===========================================================================*/
  /* Constants. */
  /*===========================================================================*/
};

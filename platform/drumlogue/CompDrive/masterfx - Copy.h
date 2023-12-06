#pragma once
/*
 *  File: master.h
 *
 *  Dummy Master Effect Class
 *
 *  Author: Etienne Noreau-Hebert <etienne@korg.co.jp>
 *
 *  2021 (c) Korg
 *
 */

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <arm_neon.h>

#include "unit.h"  // Note: Include common definitions for all units

constexpr size_t c_parameterAttack = 0;
constexpr size_t c_parameterRelease = 1;
constexpr size_t c_parameterTHold= 2;
constexpr size_t c_parameterRatio = 3;
constexpr size_t c_parameterCompType = 4;
constexpr size_t c_parameterKnee = 5;
constexpr size_t c_parameterDrive = 6;
constexpr size_t c_parameterMakeup = 7;
constexpr size_t c_parameterDryWet = 8;
constexpr size_t c_parameterGain = 9;
constexpr size_t c_parameterSidechain = 10;
constexpr size_t c_parameterNoiseTHold = 12;
constexpr size_t c_parameterNoiseRelease = 13;
constexpr size_t c_parameterNoiseGateOn = 14;



 uint32_t v_attack;
 uint32_t v_release;
 int8_t v_threshold;
 uint8_t v_ratio;
 uint16_t v_drive;
 uint16_t v_makeup;
 float v_sampleRate;
 int8_t v_compType;
 int8_t v_knee;
 int8_t v_dryWet;
 int8_t v_gain;
 uint8_t v_sidechain;
 uint8_t v_noiseThreshold;
 float v_noiseRelease;
 uint8_t v_noiseGateOn;
 float v_noiseTholdTime;
 float32x2_t envelope;

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
   v_compType = 1;
   v_noiseThreshold = 0;
   v_noiseRelease = 0;
   v_noiseGateOn = 0;

    

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


  fast_inline void Process(const float * in, float * out, size_t frames) {
       
    const float * __restrict in_p = in;
    float * __restrict out_p = out;
    const float * out_e = out_p + (frames << 1);  // assuming stereo output

    
        for (; out_p != out_e; in_p += 4, out_p += 2) {
    

        // Load a vector of four input samples (two stereo audio pairs and two sidechain pairs)
        float32x4_t inSamples = vld1q_f32(in_p);

        // Extract audio and sidechain samples using ARM NEON intrinsics
        float32x2_t audioSamples = { vgetq_lane_f32(inSamples,0),vgetq_lane_f32(inSamples,1) } ;
        float32x2_t sidechainSamples = { vgetq_lane_f32(inSamples,2),vgetq_lane_f32(inSamples,3) };

        // Apply pre-gain using ARM NEON intrinsics
        float32x2_t pregain = vdup_n_f32(1.0f + powf(10.0f, v_gain / 20.0f));
        audioSamples = vmul_f32(audioSamples, pregain);

        // Compression


        float32x2_t rectifiedAudio = vabs_f32(audioSamples);
        float32x2_t rectifiedSidechain = vabs_f32(sidechainSamples);
        
        float32x2_t attackCoefficient = vdup_n_f32(exp(-1.0f / (v_attack * v_sampleRate)));
        float32x2_t releaseCoefficient = vdup_n_f32(exp(-1.0f / (v_release * v_sampleRate)));
           
        envelope = (rectifiedAudio > envelope) ? vadd_f32(vmul_f32((1.0f - attackCoefficient), (rectifiedAudio - envelope)), envelope) : vadd_f32(vmul_f32((1.0f - releaseCoefficient), (rectifiedAudio - envelope)), envelope);

        envelope = (rectifiedSidechain > envelope) ? vadd_f32(vmul_f32((1.0f - attackCoefficient), (rectifiedSidechain - envelope)), envelope) : vadd_f32(vmul_f32((1.0f - releaseCoefficient), (rectifiedSidechain -  envelope)), envelope);

        float32x2_t threshold = vdup_n_f32(powf(10.0f, v_threshold / 20.0f));
        float32x2_t knee = vdup_n_f32(powf(10.0f, v_knee / 20.0f));
        float32x2_t slope = vdup_n_f32(1.0f / v_ratio);
        
        if (v_compType == 1) {
            float32x2_t alpha = vcge_f32(envelope, vsub_f32(threshold, knee)) ? vsub_f32(envelope, threshold) / knee : vdup_n_f32(1.0f);
            alpha = vceq_f32(alpha, vdup_n_f32(1.0f)) ? alpha : vmul_f32(vmul_n_f32(vmul_f32(alpha, alpha), 0.5f), slope);
        } else if (v_compType == 2) {
            float32x2_t alpha = vsub_f32(envelope, threshold) / knee;
            alpha = alpha / vadd_f32(knee, vdup_n_f32(1e-10f)); // Adding a small value to avoid division by zero
            alpha = vabs_f32(alpha); // Make sure alpha is positive
            alpha = vmul_f32(alpha, slope);
       

            // Soft-knee effect
            alpha = vmul_n_f32(alpha, 1.0f / v_ratio);
            alpha = vadd_f32(alpha, vdup_n_f32(1.0f));
            alpha = vrecpe_f32(alpha); */
        } 

        audioSamples *= alpha;
                
       

       if (v_drive > 0) {
            float linear_drive = pow(10.0f, -v_drive / 20.0f);
            audioSamples = vcge_f32(audioSamples, vdup_n_f32(0.0f)) ? vmin_f32(audioSamples, vdup_n_f32(linear_drive)) : vmax_f32(audioSamples, vdup_n_f32(-linear_drive));
        }

        // Noise Gate

       if (v_noiseGateOn) {
            float32x2_t noisegain = vdup_n_f32(1.0f);

            // Calculate noise_threshold using NEON intrinsics
            float32x2_t noise_threshold = vdup_n_f32(powf(10.0f, v_noiseThreshold / 20.0f));

            // Calculate release_coeff using NEON intrinsics
            float32x2_t release_coeff = vdup_n_f32(expf(-1.0f * (0.001 * v_noiseRelease / 1000.0f)));

            // Apply the noise gate and gain adjustment
            float32x2_t absAudioSamples = vabs_f32(audioSamples);
            uint32x2_t condition = vclt_f32(absAudioSamples, noise_threshold);
            noisegain = vmin_f32(noisegain, vmul_f32(noisegain, release_coeff));
            noisegain = vmul_f32(noisegain, vreinterpret_f32_u32(condition));
            audioSamples *= noisegain;
        }

        // Apply makeup gain using ARM NEON intrinsics
        float32x2_t postgain = vdup_n_f32(1.0f + powf(10.0f, v_makeup / 20.0f));
        audioSamples = vmul_f32(audioSamples, postgain);

        // Blend Dry/Wet
        float _dryWetParameter = (v_dryWet + 100.0f) / 200.0f;

        float32x2_t dryWetParameter = vdup_n_f32(_dryWetParameter);

        float32x2_t inverseDryWet = vsub_f32(vdup_n_f32(1.0f), dryWetParameter);

        float32x2_t dryMixingFactor = vmul_f32(inverseDryWet, vdup_n_f32(0.5f));

        float32x2_t wetMixingFactor = vmul_f32(dryWetParameter, vdup_n_f32(0.5f));

        float32x2_t drySamples = vmul_f32(vget_low_f32(inSamples), dryMixingFactor);

        float32x2_t wetSamples = vmul_f32(audioSamples, wetMixingFactor);

        float32x2_t outputSamples = vadd_f32(drySamples, wetSamples);
        
        vst1_f32(out_p, outputSamples);
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
        
        case c_parameterCompType:
            v_slope = value;
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
        case c_parameterCompType:
            return v_compType;
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

  /*===========================================================================*/
  /* Private Methods. */
  /*===========================================================================*/

  /*===========================================================================*/
  /* Constants. */
  /*===========================================================================*/
};

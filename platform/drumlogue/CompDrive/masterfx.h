#pragma once
/*
 *  File: masterfx.h
 *
 *  CompDrive Master Effect Class
 *
 *  Author: Evan Ravenelle
 *
 *  2023(c) Evan Ravenelle
 *
 */

#include <atomic>
#include <cmath>
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

// Parameters declared in header.c with frac=1 / frac_mode=0 carry one
// fractional bit, so the raw integer the runtime hands us is twice the
// value shown on the device. Applies to THold, Knee, Makeup, PreGain
// and NThold. Attack/Release/NRelease/Clipping are declared frac=0 and
// are used as-is.
constexpr float c_fixedPoint1BitScale = 0.5f;

// Meter row thresholds, in dB, in display order: index 0 is the TOP row of
// the 16x16 bitmap and index 15 the bottom one. Monotonically decreasing, so
// a loud signal lights every row and the bar fills from the bottom up.
// Spacing tightens toward the top where the useful resolution is.
constexpr float c_meterRowDb[16] = {
    0.0f,  -1.5f,  -3.0f,  -4.5f,  -6.0f,  -8.0f, -10.0f, -12.0f,
  -14.0f, -17.0f, -20.0f, -24.0f, -28.0f, -33.0f, -39.0f, -48.0f };

// Pixel masks for one meter row. The left 7 pixels show the left channel and
// the right 7 the right, with a 2 pixel gutter down the middle.
constexpr uint8_t c_meterLeftMask = 0x7FU;   // bits 0-6  -> pixels 0-6
constexpr uint8_t c_meterRightMask = 0xFEU;  // bits 1-7  -> pixels 9-15

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
    v_knee(0),
    v_dryWet(0),
    v_gain(0),
    v_sidechain(0),
    v_noiseThreshold(0),
    v_noiseRelease(0),
    v_noiseGateOn(0),
    envelope(vdup_n_f32(0.0f)),
    rms(vdupq_n_f32(0.0f)),
    noiseGain(1.0f),
    gateIsOpen(true),
    gateHoldCounter(0),
    envDetector(48000.0f),
    RMS(2400, 48000.0f),  // Reduced window: 50ms at 48kHz for memory efficiency
    pregainCoeff(1.0f),
    driveGain(1.0f),
    driveComp(1.0f),
    makeupCoeff(1.0f),
    gateOpenLinear(0.0f),
    gateCloseLinear(0.0f),
    gateAttackStep(1.0f),
    gateReleaseStep(0.001f),
    gateHoldSamples(0),
    invThresholdLinear(1.0f),
    slopeMinusOne(0.0f),
    halfKneeDb(0.0f),
    invTwoKneeDb(0.0f),
    attackStep(1.0f),
    releaseStep(1.0f),
    wetMix(0.5f),
    rmsBitmapIndex(0),
    meterValue(0),
    meterAttackStep(1.0f),
    meterReleaseStep(1.0f),
    meterCoeffFrames(0)
  {
    // Initialize both bitmap buffers to all off
    for (int b = 0; b < 2; b++) {
      for (int i = 0; i < 32; i++) {
        rmsBitmap[b][i] = 0x00U;
      }
    }

    for (int row = 0; row < 16; row++) {
      meterRowLinear[row] = dbToLinear(c_meterRowDb[row]);
    }
    updateCompressionCoeffs();
    updateEnvelopeCoeffs();
    updateNoiseGateCoeffs();
    updateDriveCoeffs();
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

    // Reconfigure in place; constructing replacements would put ~38KB of
    // RMS window on the stack just to copy it back out.
    envDetector.setSampleRate(v_sampleRate);
    RMS.configure(2400, v_sampleRate);  // 50ms window at 48kHz

    // Sample rate feeds the smoothing coefficients, so recompute them.
    updateEnvelopeCoeffs();
    updateNoiseGateCoeffs();

    return k_unit_err_none;
  }

  inline void Teardown() {
    // Note: cleanup and release resources if any
  }

  inline void Reset() {
    // Reset effect state
    envelope = vdup_n_f32(0.0f);
    noiseGain = 1.0f;
    gateIsOpen = true;
    gateHoldCounter = 0;
    RMS.reset();
    rms = vdupq_n_f32(0.0f);
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

  static inline float dbToLinear(float db) {
    return powf(10.0f, db * (1.0f / 20.0f));
  }

  fast_inline void Process(const float * in, float * out, size_t frames) {
    // Everything that depends only on parameters is hoisted out of the loop.
    const float32x2_t pregain     = vdup_n_f32(pregainCoeff);
    const float32x2_t makeup      = vdup_n_f32(makeupCoeff);
    const float32x2_t atkStep     = vdup_n_f32(attackStep);
    const float32x2_t relStep     = vdup_n_f32(releaseStep);
    const float32x2_t wetAmount   = vdup_n_f32(wetMix);
    const float32x2_t dryAmount   = vdup_n_f32(1.0f - wetMix);
    const float32x2_t driveIn     = vdup_n_f32(driveGain);
    const float32x2_t driveOut    = vdup_n_f32(driveComp);
    const float32x2_t ceiling     = vdup_n_f32(1.0f);
    const float32x2_t oneThird    = vdup_n_f32(1.0f / 3.0f);

    const float * __restrict in_p = in;
    float * __restrict out_p = out;
    const float * out_e = out_p + (frames << 1);  // assuming stereo output

    for (; out_p != out_e; in_p += 4, out_p += 2) {
        // Load a vector of four input samples (main L/R, sidechain L/R)
        const float32x4_t inSamples = vld1q_f32(in_p);
        const float32x2_t dryIn = vget_low_f32(inSamples);

        // Feed the RMS window first so the gate and the meter both see a
        // level that includes the current sample.
        RMS.addValue(vabsq_f32(inSamples));

        // Apply pre-gain
        float32x2_t audioSamples = vmul_f32(dryIn, pregain);

        // Update envelope follower using class member envDetector
        envelope = envDetector.processSample(inSamples, envelope, v_sidechain,
                                             atkStep, relStep);

        // Stereo-linked detection: derive one gain from the louder channel so
        // that compression cannot shift the stereo image.
        const float envLinked = vget_lane_f32(vpmax_f32(envelope, envelope), 0);
        audioSamples = vmul_f32(audioSamples,
                                vdup_n_f32(computeCompressionGain(envLinked)));

        // Drive / clipping.
        //
        // Two changes from a plain hard clip, both because the control was
        // inaudible over most of its range:
        //
        //  - Gain INTO a fixed ceiling and compensate on the way out, rather
        //    than lowering a ceiling toward a signal whose level we cannot
        //    know. A ceiling referenced to full scale does nothing at all
        //    until it drops below the source peak, which killed the bottom
        //    half of the knob on any real-world material.
        //
        //  - Cubic soft clip rather than a hard clamp. A hard clamp has a
        //    threshold: below it the signal is untouched, above it the level
        //    collapses. The cubic curve bends from the very first degree of
        //    drive, so the knob does something immediately, and it preserves
        //    far more RMS so what you hear is harmonics rather than a fade.
        //
        //    y = x - x^3/3, clamped to |x| <= 1, saturating at +/-2/3 with
        //    unity slope through the origin.
        if (v_drive > 0) {
            float32x2_t x = vmul_f32(audioSamples, driveIn);
#ifdef CD_HOST_DIAG
            // How much of the signal the drive stage is actually saturating,
            // as opposed to the output simply running hot. Host bench only -
            // the device build carries none of this.
            diagSatAccum = vadd_f32(
                diagSatAccum,
                vbsl_f32(vcge_f32(vabs_f32(x), ceiling), vdup_n_f32(1.0f), vdup_n_f32(0.0f)));
            diagSamples += 2;
#endif
            x = vmin_f32(vmax_f32(x, vneg_f32(ceiling)), ceiling);
            const float32x2_t cubed = vmul_f32(x, vmul_f32(x, x));
            x = vsub_f32(x, vmul_f32(cubed, oneThird));
            audioSamples = vmul_f32(x, driveOut);
        }

        // Noise Gate
        //
        // Keyed off the windowed RMS rather than the instantaneous sample.
        // Testing |sample| against the threshold re-triggers at every zero
        // crossing - twice per cycle of the signal - which is what made the
        // gate grainy. The 50ms RMS window rides over the waveform and
        // decays smoothly through a tail.
        //
        // Two further guards against chatter: separate open/close thresholds
        // (hysteresis), so a level hovering at the threshold cannot toggle
        // the gate; and a hold time, so a brief dip inside a decaying tail
        // does not slam it shut.
        if (v_noiseGateOn) {
            // Stereo-linked: one decision, one gain, both channels.
            const float32x2_t rmsPair = vget_low_f32(RMS.currentLevel());
            const float rmsLevel = vget_lane_f32(vpmax_f32(rmsPair, rmsPair), 0);

            if (rmsLevel >= gateOpenLinear) {
                gateIsOpen = true;
                gateHoldCounter = gateHoldSamples;
            } else if (rmsLevel < gateCloseLinear) {
                // Below the close threshold: run out the hold, then release.
                if (gateHoldCounter > 0) {
                    --gateHoldCounter;
                } else {
                    gateIsOpen = false;
                }
            }
            // Between the two thresholds the current state is retained.

            const float target = gateIsOpen ? 1.0f : 0.0f;
            const float step = gateIsOpen ? gateAttackStep : gateReleaseStep;
            noiseGain += (target - noiseGain) * step;

            audioSamples = vmul_f32(audioSamples, vdup_n_f32(noiseGain));
        }

        // Apply makeup gain
        audioSamples = vmul_f32(audioSamples, makeup);

        // Blend Dry/Wet: dry * (1 - mix) + wet * mix
        const float32x2_t outputSamples =
            vmla_f32(vmul_f32(dryIn, dryAmount), audioSamples, wetAmount);

        // Store output
        vst1_f32(out_p, outputSamples);
    }

#ifdef CD_HOST_DIAG
    {
      const float n = static_cast<float>(diagSamples);
      const float sat = vget_lane_f32(diagSatAccum, 0) + vget_lane_f32(diagSatAccum, 1);
      diagDriveSaturated = (n > 0.0f) ? (sat / n) : 0.0f;
      diagSatAccum = vdup_n_f32(0.0f);
      diagSamples = 0;
    }
#endif

    // Update RMS bitmap once per buffer (not every sample!)
    //
    // The envelope advances one step per buffer, so the time constants are
    // scaled by the buffer length. Meter ballistics are deliberately fixed
    // and independent of the compressor's Attack/Release knobs.
    updateMeterCoeffs(static_cast<uint32_t>(frames));
    rms = RMS.getRMS(meterAttackStep, meterReleaseStep);

    // Write to the buffer the UI is not holding, then publish it.
    const uint32_t back = 1U - rmsBitmapIndex.load(std::memory_order_relaxed);
    const float32x2_t level = vget_low_f32(rms);
    updateBitmapWithRMS(level, rmsBitmap[back]);
    rmsBitmapIndex.store(back, std::memory_order_release);

    // Also publish the level as a 0..15 parameter value. The runtime repaints
    // a parameter only when its value changes, so reporting a moving value is
    // what makes the meter animate. Quantising to the 16 meter rows also rate
    // limits the repaints to something the display can keep up with.
    meterValue.store(meterLevelFrom(level), std::memory_order_relaxed);
  }

  // Build the 16x16 1bpp meter bitmap for the current level.
  //
  // Format per the drumlogue SDK: 32 bytes, two bytes per row, row 0 at the
  // top, each byte read least-significant-bit first for left-to-right
  // pixels. All 16 rows are written every update.
  // Number of meter rows lit by the louder channel, 0..15.
  inline int32_t meterLevelFrom(float32x2_t rmsValue) const {
    const float linked = vget_lane_f32(vpmax_f32(rmsValue, rmsValue), 0);
    int32_t lit = 0;
    for (int row = 15; row >= 0; --row) {
      if (linked >= meterRowLinear[row]) ++lit;
      else break;
    }
    return lit > 15 ? 15 : lit;
  }

  inline void updateBitmapWithRMS(float32x2_t rmsValue, uint8_t* bitmap) {
    const float levelL = vget_lane_f32(rmsValue, 0);
    const float levelR = vget_lane_f32(rmsValue, 1);

    for (int row = 0; row < 16; ++row) {
      const float rowLevel = meterRowLinear[row];
      bitmap[row * 2]     = (levelL >= rowLevel) ? c_meterLeftMask : 0x00U;
      bitmap[row * 2 + 1] = (levelR >= rowLevel) ? c_meterRightMask : 0x00U;
    }
  }

  inline void setParameter(uint8_t index, int32_t value) {
    switch (index) {
        case c_parameterAttack:
            v_attack = value;
            updateEnvelopeCoeffs();
            break;

        case c_parameterRelease:
            v_release = value;
            updateEnvelopeCoeffs();
            break;

        case c_parameterTHold:
            v_threshold = value;
            updateCompressionCoeffs();
            break;

        case c_parameterRatio:
            v_ratio = value;
            updateCompressionCoeffs();
            break;

        case c_parameterKnee:
            v_knee = value;
            updateCompressionCoeffs();
            break;

        case c_parameterRMSBitmap:
            // Display-only; the unit owns this value, so ignore writes rather
            // than let the encoder fight the meter.
            break;

        case c_parameterDrive:
            v_drive = value;
            updateDriveCoeffs();
            break;

        case c_parameterMakeup:
            v_makeup = value;
            makeupCoeff = dbToLinear(static_cast<float>(value) * c_fixedPoint1BitScale);
            break;

        case c_parameterDryWet:
            v_dryWet = value;
            // -100 = fully dry, 0 = balanced, +100 = fully wet.
            wetMix = (static_cast<float>(value) + 100.0f) * (1.0f / 200.0f);
            break;

        case c_parameterGain:
            v_gain = value;
            pregainCoeff = dbToLinear(static_cast<float>(value) * c_fixedPoint1BitScale);
            break;

        case c_parameterSidechain:
            v_sidechain = value;
            break;

        case c_parameterNoiseTHold:
            v_noiseThreshold = value;
            updateNoiseGateCoeffs();
            break;

        case c_parameterNoiseRelease:
            v_noiseRelease = value;
            updateNoiseGateCoeffs();
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
        case c_parameterRMSBitmap:
            // Reporting a changing value is what drives the repaint.
            return meterValue.load(std::memory_order_relaxed);
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
            // Read whichever buffer the audio thread last published.
            return rmsBitmap[rmsBitmapIndex.load(std::memory_order_acquire)];

      // Note: Bitmap memory must be accessible even after function returned.
      //       It can be assumed that caller will have copied or used the bitmap
      //       before the next call to getParameterBmpValue
      // Note: Not yet implemented upstream

      default:
        break;
    }
    return nullptr;
  }

#ifdef CD_HOST_DIAG
  // Fraction of samples the drive stage clipped in the last buffer, 0..1.
  inline float getDriveSaturation() const { return diagDriveSaturated; }
#endif

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

  // Raw parameter values, as handed to us by the runtime.
  int32_t v_attack;
  int32_t v_release;
  int32_t v_threshold;
  int32_t v_ratio;
  int32_t v_drive;
  int32_t v_makeup;
  float v_sampleRate;
  int32_t v_knee;
  int32_t v_dryWet;
  int32_t v_gain;
  int32_t v_sidechain;
  int32_t v_noiseThreshold;
  int32_t v_noiseRelease;
  int32_t v_noiseGateOn;

  // NEON constants and state
  float32x2_t envelope;
  float32x4_t rms;

  // Noise gate state. The gain is scalar because the gate is stereo-linked.
  float noiseGain;
  bool gateIsOpen;          // hysteresis latch
  uint32_t gateHoldCounter; // samples left to hold before releasing

  // DSP objects
  EnvelopeDetector envDetector;
  RMSCalculator RMS;

  // Pre-calculated coefficients (to avoid expensive operations in audio loop)
  float pregainCoeff;
  float driveGain;   // gain into the clipper
  float driveComp;   // output compensation, keeps loudness roughly constant
  float makeupCoeff;

  // Noise gate coefficients.
  float gateOpenLinear;      // level at or above which the gate opens
  float gateCloseLinear;     // level below which it may start to close
  float gateAttackStep;      // per-sample approach rate while opening
  float gateReleaseStep;     // per-sample approach rate while closing
  uint32_t gateHoldSamples;  // hold time, in samples

  // Compression curve coefficients, derived from threshold / ratio / knee.
  float invThresholdLinear;  // 1 / threshold, to get overshoot by multiply
  float slopeMinusOne;       // (1/ratio) - 1, always <= 0
  float halfKneeDb;          // knee width / 2, in dB
  float invTwoKneeDb;        // 1 / (2 * knee width), in dB

  // Envelope smoothing steps, i.e. 1 - exp(-1 / (seconds * samplerate)).
  float attackStep;
  float releaseStep;

  // Normalized wet amount, 0.0 = fully dry, 1.0 = fully wet.
  float wetMix;

#ifdef CD_HOST_DIAG
  float32x2_t diagSatAccum = vdup_n_f32(0.0f);
  uint32_t diagSamples = 0;
  float diagDriveSaturated = 0.0f;   // 0..1, fraction of samples in saturation
#endif

  // Display bitmap. Double buffered: the audio thread fills one while the UI
  // thread may still be reading the other, with the index published on
  // release/acquire so the UI never sees a half-written meter.
  uint8_t rmsBitmap[2][32];
  std::atomic<uint32_t> rmsBitmapIndex;

  // Current meter level, 0..15, reported as the bitmap parameter's value.
  std::atomic<int32_t> meterValue;

  // Row thresholds as linear amplitudes, derived from c_meterRowDb.
  float meterRowLinear[16];

  // Meter envelope steps, per buffer, cached against the frame count they
  // were computed for.
  float meterAttackStep;
  float meterReleaseStep;
  uint32_t meterCoeffFrames;

  /*===========================================================================*/
  /* Private Methods. */
  /*===========================================================================*/

  // Compressor gain for one linked envelope sample.
  //
  // Working in dB against the *overshoot* is what makes this a compressor:
  // gain reduction has to grow as the signal pushes further past the
  // threshold. The quadratic middle segment gives a soft knee that joins
  // both straight segments continuously in level and in slope.
  //
  //   over  = 20*log10(env / threshold)
  //   gr    = 0                                   over <= -W/2
  //         = (1/R - 1) * (over + W/2)^2 / (2W)   |over| <  W/2
  //         = (1/R - 1) * over                    over >=  W/2
  //
  fast_inline float computeCompressionGain(float env) const {
    if (env <= k_silenceFloor) {
      return 1.0f;  // nothing to compress
    }

    const float overDb = 20.0f * log10f(env * invThresholdLinear);

    float grDb;
    if (overDb <= -halfKneeDb) {
      grDb = 0.0f;  // below the knee, unity gain
    } else if (overDb >= halfKneeDb) {
      grDb = slopeMinusOne * overDb;  // fully compressed, slope 1/R
    } else {
      const float kneeOffset = overDb + halfKneeDb;
      grDb = slopeMinusOne * kneeOffset * kneeOffset * invTwoKneeDb;
    }

    return dbToLinear(grDb);
  }

  inline void updateCompressionCoeffs() {
    const float thresholdDb = static_cast<float>(v_threshold) * c_fixedPoint1BitScale;
    const float kneeDb = static_cast<float>(v_knee) * c_fixedPoint1BitScale;
    const float ratio = (v_ratio > 0) ? static_cast<float>(v_ratio) : 1.0f;

    invThresholdLinear = dbToLinear(-thresholdDb);
    slopeMinusOne = (1.0f / ratio) - 1.0f;
    halfKneeDb = kneeDb * 0.5f;
    invTwoKneeDb = (kneeDb > 0.0f) ? (1.0f / (2.0f * kneeDb)) : 0.0f;
  }

  inline void updateEnvelopeCoeffs() {
    attackStep = smoothingStep(static_cast<float>(v_attack));
    releaseStep = smoothingStep(static_cast<float>(v_release));
  }

  // Approach rate for a time constant applied once per buffer of `frames`
  // samples, rather than once per sample.
  inline float bufferStep(float milliseconds, uint32_t frames) const {
    if (milliseconds <= 0.0f || v_sampleRate <= 0.0f || frames == 0) {
      return 1.0f;
    }
    return 1.0f -
           expf(-static_cast<float>(frames) / ((milliseconds / 1000.0f) * v_sampleRate));
  }

  // Recomputed only when the buffer length changes, which in practice is
  // once.
  inline void updateMeterCoeffs(uint32_t frames) {
    if (frames == meterCoeffFrames) {
      return;
    }
    meterCoeffFrames = frames;
    meterAttackStep = bufferStep(k_meterAttackMs, frames);
    meterReleaseStep = bufferStep(k_meterReleaseMs, frames);
  }

  // Hard clipping removes energy, so compensating fully by 1/driveGain would
  // leave the effect sounding like a volume drop. k_driveCompExponent scales
  // the compensation so perceived loudness stays roughly flat across the
  // range and what you hear is the added harmonics. Tuned by measurement -
  // see the drive sweep in test/host/reference.
  inline void updateDriveCoeffs() {
    const float driveDb = static_cast<float>(v_drive);   // frac=0, raw dB
    driveGain = dbToLinear(driveDb);
    // k_softClipPeak undoes the curve's 2/3 saturation ceiling; the exponent
    // then gives back only part of the drive gain, which is what keeps
    // loudness roughly flat instead of climbing.
    driveComp = dbToLinear(-driveDb * k_driveCompExponent);
  }

  inline void updateNoiseGateCoeffs() {
    const float thresholdDb = static_cast<float>(v_noiseThreshold) * c_fixedPoint1BitScale;

    gateOpenLinear = dbToLinear(thresholdDb);
    gateCloseLinear = dbToLinear(thresholdDb - k_gateHysteresisDb);

    gateAttackStep = smoothingStep(k_gateAttackMs);
    gateReleaseStep = smoothingStep(static_cast<float>(v_noiseRelease));

    gateHoldSamples =
        static_cast<uint32_t>((k_gateHoldMs / 1000.0f) * v_sampleRate);
  }

  // Per-sample approach rate for a given time constant in milliseconds.
  inline float smoothingStep(float milliseconds) const {
    if (milliseconds <= 0.0f || v_sampleRate <= 0.0f) {
      return 1.0f;  // instantaneous
    }
    return 1.0f - expf(-1.0f / ((milliseconds / 1000.0f) * v_sampleRate));
  }

  /*===========================================================================*/
  /* Constants. */
  /*===========================================================================*/

  // Envelope level below which the compressor is a no-op. Sits well under
  // any audible signal but keeps log10f away from zero.
  static constexpr float k_silenceFloor = 1e-9f;

  // Gate opens this far below the NThold setting before it will close
  // again. Standard hysteresis; stops a level sitting on the threshold from
  // toggling the gate.
  static constexpr float k_gateHysteresisDb = 6.0f;

  // Gate fade-in. Fast enough to keep a drum transient intact, slow enough
  // not to click. Closing is governed by the NRelease parameter.
  static constexpr float k_gateAttackMs = 1.0f;

  // How long the gate stays open after the level falls below the close
  // threshold. Bridges brief dips inside a decaying tail.
  static constexpr float k_gateHoldMs = 30.0f;

  // Fraction of the drive gain taken back out after the soft clip. Tuned by
  // measurement so output RMS stays roughly flat as drive rises; see
  // updateDriveCoeffs().
  // Fraction of the drive gain taken back out after the soft clip.
  //
  // Measured, not guessed: swept 0.0 / 0.1 / 0.2 / 0.3 against a -6 dBFS
  // sine and picked the flattest output RMS across the knob's active range.
  // 0.0 climbs 5.3 dB (reads as "louder" rather than "dirtier"); 0.2 holds
  // within 1.9 dB and eases down slightly at the top, which offsets the
  // extra perceived brightness the harmonics add.
  static constexpr float k_driveCompExponent = 0.20f;

  // Meter ballistics: quick to catch a hit, slow enough to read.
  static constexpr float k_meterAttackMs = 10.0f;
  static constexpr float k_meterReleaseMs = 400.0f;
};

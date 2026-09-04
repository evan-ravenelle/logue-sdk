#ifndef ENVELOPE_H
#define ENVELOPE_H

#include <arm_neon.h>

// Peak envelope follower with separate attack/release ballistics.
//
// Smoothing coefficients are supplied by the caller already converted to
// per-sample step values (1 - exp(-1 / (seconds * samplerate))), so that no
// transcendental math is performed in the render loop. See
// MasterFX::updateEnvelopeCoeffs().
class EnvelopeDetector {
public:
    explicit EnvelopeDetector(float sampleRate) : sampleRate_(sampleRate) {}

    void setSampleRate(float sampleRate) { sampleRate_ = sampleRate; }
    float sampleRate() const { return sampleRate_; }

    inline float32x2_t processSample(float32x4_t input,
                                     float32x2_t envState,
                                     int sidechainOn,
                                     float32x2_t attackStep,
                                     float32x2_t releaseStep) const {
        // Input frame layout: [ main_left, main_right, sidechain_left, sidechain_right ]
        const float32x2_t rectified = (sidechainOn == 1)
                                          ? vabs_f32(vget_high_f32(input))
                                          : vabs_f32(vget_low_f32(input));

        // Rising signal attacks, falling signal releases.
        const uint32x2_t rising = vcgt_f32(rectified, envState);
        const float32x2_t step = vbsl_f32(rising, attackStep, releaseStep);

        const float32x2_t delta = vsub_f32(rectified, envState);
        return vadd_f32(envState, vmul_f32(delta, step));
    }

private:
    float sampleRate_;
};

#endif // ENVELOPE_H

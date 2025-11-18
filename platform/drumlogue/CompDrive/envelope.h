#ifndef ENVELOPE_H
#define ENVELOPE_H

#include <cmath>
#include <arm_neon.h>

class EnvelopeDetector {
public:
    EnvelopeDetector(float sampleRate)
        : sampleRate_(sampleRate) {
    }

    float32x2_t processSample(float32x4_t input, float32x2_t& envState, int sidechainOn, float attackTime, float releaseTime) {
        // Calculate envelope coefficients (ideally these should be pre-calculated in setParameter)
        attackCoefficient_ = vdup_n_f32(expf(-1.0f / (attackTime / 1000.0f * sampleRate_ + 1e-6f)));
        releaseCoefficient_ = vdup_n_f32(expf(-1.0f / (releaseTime / 1000.0f * sampleRate_ + 1e-6f)));

        // Rectify the inputs (convert to absolute value)
        float32x2_t rectifiedInput;
        if (sidechainOn == 1) {
            rectifiedInput = vabs_f32(vget_high_f32(input));
        } else {
            rectifiedInput = vabs_f32(vget_low_f32(input));
        }

        // Calculate the envelope based on the input
        // If rectified input > current envelope, we're attacking (rising)
        uint32x2_t attack = vcgt_f32(rectifiedInput, envState);
        float32x2_t coeff = vbsl_f32(attack, attackCoefficient_, releaseCoefficient_);

        // Envelope smoothing: envState += (rectifiedInput - envState) * (1 - coeff)
        // Fixed: Use vsub_f32 instead of '-' operator
        float32x2_t delta = vsub_f32(rectifiedInput, envState);
        float32x2_t oneMinusCoeff = vsub_f32(vdup_n_f32(1.0f), coeff);
        envState = vadd_f32(envState, vmul_f32(delta, oneMinusCoeff));

        return envState;
    }

private:
    float attackTime_;
    float releaseTime_;
    float sampleRate_;
    float32x2_t attackCoefficient_;
    float32x2_t releaseCoefficient_;
 };


#endif // ENVELOPE_H

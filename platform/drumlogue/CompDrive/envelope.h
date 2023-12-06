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
        attackCoefficient_ = vdup_n_f32(exp(-1.0f / (attackTime/1000.0f * sampleRate_)));
        releaseCoefficient_ = vdup_n_f32(exp(-1.0f / (releaseTime/1000.0f * sampleRate_)));
        
        // Rectify the inputs (convert to absolute value)
        float32x2_t rectifiedInput; 
        if (sidechainOn == 1) {
             rectifiedInput = vabs_f32(vget_high_f32(input));
        } else {
             rectifiedInput = vabs_f32(vget_low_f32(input));
        }

        // Calculate the envelope based on the main input
        uint32x2_t attack = vcgt_f32(rectifiedInput, envState); // if rectifiedInput < envelope, signal is attacking
        float32x2_t coeff = vbsl_f32(attack, attackCoefficient_, releaseCoefficient_);

        envState = vadd_f32(vmul_f32(vsub_f32(vdup_n_f32(1.0f), coeff), (rectifiedInput - envState)), envState);
      

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

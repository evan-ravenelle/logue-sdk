#ifndef RMS_H
#define RMS_H

#include <cmath>
#include <deque>

#include <arm_neon.h>

class RMSCalculator {
public:
     RMSCalculator(size_t rmsWindow, float sampleRate)
        : rmsWindow(12000.0f), maxLength(maxLength), sumOfSquares(vdupq_n_f32(0e-10f)), envelope(vdupq_n_f32(0e-10f)), sampleRate(48000.0f) {}

    void addValue(float32x4_t value) {
        // Add the value to the end of the list
        data.push_back(value);

        // Check if the list exceeds the specified length
        if (data.size() > rmsWindow) {
            // Remove the oldest entry
            float32x4_t oldest = data.front();
            data.pop_front();

            // Update the sum of squares after removing the oldest entry
            sumOfSquares = vsubq_f32(sumOfSquares, vmulq_f32(oldest, oldest));
        }


        sumOfSquares = vaddq_f32(sumOfSquares, vmulq_f32(value, value));
        meanOfSquares = sumOfSquares / vdupq_n_f32(data.size());
        currentRMS = vrsqrteq_f32(meanOfSquares);
        
        
    }
    float32x4_t getRMS(float attackTime, float releaseTime) {
        envelope = calculateEnvelope(envelope, currentRMS, sampleRate, attackTime / 1000.0f, releaseTime / 1000.0f);
        return envelope;
    }


private:
    std::deque<float32x4_t> data; // Store the values in an ordered list
    size_t maxLength;       // Maximum length of the list
    float32x4_t sumOfSquares;    // Sum of squares for efficient RMS calculation
    float32x4_t meanOfSquares;
    float32x4_t currentRMS;
    float32x4_t envelope;        // Envelope to simulate attack and release

    float attackTime;            // Attack time constant
    float releaseTime;           // Release time constant
    float sampleRate;
    float rmsWindow;

    float32x4_t calculateEnvelope(float32x4_t currentEnvelope, float32x4_t currentRMS, float sampleRate,  float attackTime, float releaseTime) {
        // Calculate the envelope based on attack and release time constants
        float32x4_t delta = vsubq_f32(currentRMS, currentEnvelope);
        float32x4_t attackCoefficient = vdupq_n_f32(expf(-1.0f / (attackTime * sampleRate))); // Calculate attack coefficient
        float32x4_t releaseCoefficient = vdupq_n_f32(expf(-1.0f / (releaseTime * sampleRate))); // Calculate release coefficient
        float32x4_t attackRelease = vbslq_f32(vcleq_f32(delta, vdupq_n_f32(0.0f)), attackCoefficient, releaseCoefficient);

        return vaddq_f32(currentEnvelope, vmulq_f32(delta, attackRelease));
    }
};

#endif // RMS_H
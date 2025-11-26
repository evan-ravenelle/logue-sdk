#ifndef RMS_H
#define RMS_H

#include <cmath>
#include <arm_neon.h>

// Fixed-size circular buffer RMS calculator for real-time audio
// Uses NEON intrinsics for SIMD optimization
// Memory optimized: 2400 samples = 50ms at 48kHz = ~38KB
class RMSCalculator {
public:
    static constexpr size_t MAX_WINDOW_SIZE = 2400;  // 50ms at 48kHz (reduced for memory efficiency)

    RMSCalculator(size_t windowSize, float sampleRate)
        : rmsWindow(windowSize > MAX_WINDOW_SIZE ? MAX_WINDOW_SIZE : windowSize),
          sampleRate(sampleRate),
          writeIndex(0),
          currentSize(0),
          sumOfSquares(vdupq_n_f32(1e-10f)),  // Small epsilon to avoid divide by zero
          currentRMS(vdupq_n_f32(0.0f)),
          envelope(vdupq_n_f32(0.0f))
    {
        // Initialize circular buffer to zero
        for (size_t i = 0; i < MAX_WINDOW_SIZE; i++) {
            circularBuffer[i] = vdupq_n_f32(0.0f);
        }
    }

    void addValue(float32x4_t value) {
        // If buffer is full, subtract the oldest value from sum of squares
        if (currentSize >= rmsWindow) {
            float32x4_t oldest = circularBuffer[writeIndex];
            sumOfSquares = vsubq_f32(sumOfSquares, vmulq_f32(oldest, oldest));
        } else {
            currentSize++;
        }

        // Add new value to circular buffer
        circularBuffer[writeIndex] = value;

        // Add new value squared to sum
        sumOfSquares = vaddq_f32(sumOfSquares, vmulq_f32(value, value));

        // Advance write index (circular)
        writeIndex++;
        if (writeIndex >= rmsWindow) {
            writeIndex = 0;
        }

        // Calculate mean of squares
        float32x4_t divisor = vdupq_n_f32(static_cast<float>(currentSize > 0 ? currentSize : 1));

        // NEON doesn't have native division, use reciprocal estimate + Newton-Raphson refinement
        float32x4_t reciprocal = vrecpeq_f32(divisor);
        reciprocal = vmulq_f32(vrecpsq_f32(divisor, reciprocal), reciprocal);  // One refinement step

        float32x4_t meanOfSquares = vmulq_f32(sumOfSquares, reciprocal);

        // Calculate RMS: sqrt(meanOfSquares)
        // Use reciprocal square root estimate + refinement, then take reciprocal
        float32x4_t rsqrt = vrsqrteq_f32(meanOfSquares);
        rsqrt = vmulq_f32(vrsqrtsq_f32(vmulq_f32(meanOfSquares, rsqrt), rsqrt), rsqrt);  // Refinement

        // currentRMS = meanOfSquares * rsqrt = sqrt(meanOfSquares)
        currentRMS = vmulq_f32(meanOfSquares, rsqrt);
    }

    float32x4_t getRMS(float attackTime, float releaseTime) {
        // Apply envelope follower with attack/release
        envelope = calculateEnvelope(envelope, currentRMS, sampleRate,
                                     attackTime / 1000.0f, releaseTime / 1000.0f);
        return envelope;
    }

    void reset() {
        writeIndex = 0;
        currentSize = 0;
        sumOfSquares = vdupq_n_f32(1e-10f);
        currentRMS = vdupq_n_f32(0.0f);
        envelope = vdupq_n_f32(0.0f);

        for (size_t i = 0; i < MAX_WINDOW_SIZE; i++) {
            circularBuffer[i] = vdupq_n_f32(0.0f);
        }
    }

private:
    float32x4_t circularBuffer[MAX_WINDOW_SIZE];  // Fixed-size circular buffer
    size_t rmsWindow;          // Active window size
    float sampleRate;
    size_t writeIndex;         // Current write position
    size_t currentSize;        // Current buffer fill level
    float32x4_t sumOfSquares;  // Running sum of squares
    float32x4_t currentRMS;    // Current RMS value
    float32x4_t envelope;      // Envelope follower output

    float32x4_t calculateEnvelope(float32x4_t currentEnvelope, float32x4_t targetRMS,
                                   float sampleRate, float attackTime, float releaseTime) {
        // Calculate the envelope based on attack and release time constants
        float32x4_t delta = vsubq_f32(targetRMS, currentEnvelope);

        // Calculate coefficients
        float attackCoeff = expf(-1.0f / (attackTime * sampleRate + 1e-6f));
        float releaseCoeff = expf(-1.0f / (releaseTime * sampleRate + 1e-6f));

        float32x4_t attackCoeffVec = vdupq_n_f32(attackCoeff);
        float32x4_t releaseCoeffVec = vdupq_n_f32(releaseCoeff);

        // Choose attack or release based on whether signal is rising or falling
        uint32x4_t risingMask = vcgtq_f32(delta, vdupq_n_f32(0.0f));
        float32x4_t coeffToUse = vbslq_f32(risingMask, attackCoeffVec, releaseCoeffVec);

        // Apply envelope smoothing: env += delta * (1 - coeff)
        float32x4_t oneMinusCoeff = vsubq_f32(vdupq_n_f32(1.0f), coeffToUse);
        return vaddq_f32(currentEnvelope, vmulq_f32(delta, oneMinusCoeff));
    }
};

#endif // RMS_H

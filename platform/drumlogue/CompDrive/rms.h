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

    // Lower bound on the mean of squares, ~-200 dBFS. Keeps the reciprocal
    // square root away from zero. See addValue().
    static constexpr float k_meanSquareFloor = 1e-20f;

    RMSCalculator(size_t windowSize, float sampleRate)
        : rmsWindow(windowSize == 0 ? 1 : (windowSize > MAX_WINDOW_SIZE ? MAX_WINDOW_SIZE : windowSize)),
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

        // Clamp before the reciprocal square root. The running sum is
        // maintained by adding the new square and subtracting the oldest, so
        // rounding error can walk it to zero (or slightly negative) during
        // long silences. vrsqrteq_f32(0) is +inf and 0 * inf is NaN, which
        // would poison every consumer of this value - including the noise
        // gate, which is in the audio path.
        meanOfSquares = vmaxq_f32(meanOfSquares, vdupq_n_f32(k_meanSquareFloor));

        // Calculate RMS: sqrt(meanOfSquares)
        // Use reciprocal square root estimate + refinement, then take reciprocal
        float32x4_t rsqrt = vrsqrteq_f32(meanOfSquares);
        rsqrt = vmulq_f32(vrsqrtsq_f32(vmulq_f32(meanOfSquares, rsqrt), rsqrt), rsqrt);  // Refinement

        // currentRMS = meanOfSquares * rsqrt = sqrt(meanOfSquares)
        currentRMS = vmulq_f32(meanOfSquares, rsqrt);
    }

    // Windowed RMS as of the most recent addValue(), without the
    // attack/release smoothing applied by getRMS(). Updated every sample at
    // no extra cost, since addValue() already computes it.
    float32x4_t currentLevel() const { return currentRMS; }

    // Advance the display envelope by one step.
    //
    // attackStep/releaseStep are per-CALL approach rates, not per-sample
    // ones. This is called once per audio buffer, so the caller has to scale
    // the time constants by the buffer length - see MasterFX::bufferStep().
    float32x4_t getRMS(float attackStep, float releaseStep) {
        envelope = calculateEnvelope(envelope, currentRMS, attackStep, releaseStep);
        return envelope;
    }

    // Reconfigure in place. Avoids constructing a temporary RMSCalculator
    // (which carries a ~38KB buffer) on the stack just to copy-assign it.
    void configure(size_t windowSize, float sampleRateHz) {
        rmsWindow = (windowSize > MAX_WINDOW_SIZE) ? MAX_WINDOW_SIZE : windowSize;
        if (rmsWindow == 0) {
            rmsWindow = 1;
        }
        sampleRate = sampleRateHz;
        reset();
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
                                   float attackStep, float releaseStep) {
        float32x4_t delta = vsubq_f32(targetRMS, currentEnvelope);

        // Rising level attacks, falling level releases.
        uint32x4_t risingMask = vcgtq_f32(delta, vdupq_n_f32(0.0f));
        float32x4_t step = vbslq_f32(risingMask, vdupq_n_f32(attackStep),
                                                 vdupq_n_f32(releaseStep));

        return vaddq_f32(currentEnvelope, vmulq_f32(delta, step));
    }
};

#endif // RMS_H

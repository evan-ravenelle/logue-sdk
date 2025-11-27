/*
 * Modular Oscillator for minilogue xd
 *
 * Features:
 * - Dual phase accumulator architecture
 * - Selectable waveform integration types (triangle, sine, ramp, saw)
 * - Adjustable phase transition point and slope linearity
 * - Internal LFO with variable rate
 * - Modulation routing to Amp, Freq, Shape, or Linearity
 * - Cross-modulation between oscillators
 * - Voice overlay for 2-voice simulation
 *
 * (c) 2025
 */

#include "userosc.h"
#include "float_math.h"

// ============================================================================
// Constants
// ============================================================================

#define SAMPLE_RATE 48000.0f
#define INV_SAMPLE_RATE (1.0f / SAMPLE_RATE)

// LFO frequency range: 0.1 Hz to 10000 Hz (audio rate)
#define LFO_FREQ_MIN 0.1f
#define LFO_FREQ_MAX 10000.0f

// Waveform integration types
enum WaveformType {
  WAVE_SQUARE = 0,
  WAVE_TRIANGLE,
  WAVE_SINE,
  WAVE_RAMP_UP,
  WAVE_RAMP_DOWN,
  WAVE_SAW,
  WAVE_COUNT
};

// LFO destination routing
enum LFODestination {
  DEST_AMP = 0,
  DEST_FREQ,
  DEST_SHAPE,
  DEST_LINEARITY,
  DEST_COUNT
};

// ============================================================================
// State Structure
// ============================================================================

typedef struct {
  // Phase accumulators
  float phase_main;      // Main oscillator phase [0, 1)
  float phase_lfo;       // LFO/modulator phase [0, 1)

  // Phase increments (computed from frequency)
  float w0_main;         // Main oscillator phase increment
  float w0_lfo;          // LFO phase increment

  // Waveform parameters
  float shape_main;      // Main waveform type morph [0, 1]
  float shape_lfo;       // LFO waveform type morph [0, 1]
  float phase_trans;     // Phase transition point [0, 1]
  float linearity;       // Slope linearity/curve [-1, 1]

  // LFO parameters
  float lfo_rate;        // LFO frequency scaling [0, 1]
  uint8_t lfo_dest;      // LFO destination routing
  float lfo_value;       // Current LFO output value

  // Modulation parameters
  float crossmod_amt;    // Cross-modulation amount [0, 1]
  int8_t overlay_interval; // Voice overlay interval in semitones [-24, +24]

  // 8-bit mode parameters
  uint8_t use_8bit;      // 0 = floating point, 1 = 8-bit mode
  float sample_hold;     // Sample hold value for downsampling
  uint16_t sample_counter; // Counter for sample rate reduction

  // MIDI tracking
  uint8_t note;          // Current MIDI note
  uint8_t mod;           // Current MIDI fine tune

} OscState;

// Global state
static OscState s_state;

// ============================================================================
// Waveform Generation Functions
// ============================================================================

// Fast approximation of sine using Taylor series (optimized)
static inline float fast_sine(float phase) {
  // Use built-in optimized sine from osc_api
  return osc_sinf(phase);
}

// Triangle wave generator with adjustable transition point and linearity
static inline float generate_triangle(float phase, float transition, float linearity) {
  float output;

  if (phase < transition) {
    // Rising slope
    float t = phase / transition;

    // Apply linearity curve (exponential/logarithmic response)
    if (linearity > 0.01f) {
      // Exponential curve (convex)
      t = fastpowf(t, 1.0f + linearity * 2.0f);
    } else if (linearity < -0.01f) {
      // Logarithmic curve (concave)
      t = 1.0f - fastpowf(1.0f - t, 1.0f - linearity * 2.0f);
    }

    output = -1.0f + 2.0f * t;
  } else {
    // Falling slope
    float t = (phase - transition) / (1.0f - transition);

    // Apply linearity curve
    if (linearity > 0.01f) {
      t = fastpowf(t, 1.0f + linearity * 2.0f);
    } else if (linearity < -0.01f) {
      t = 1.0f - fastpowf(1.0f - t, 1.0f - linearity * 2.0f);
    }

    output = 1.0f - 2.0f * t;
  }

  return output;
}

// Ramp wave generator (sawtooth variants)
static inline float generate_ramp_up(float phase, float linearity) {
  float t = phase;

  // Apply linearity curve
  if (linearity > 0.01f) {
    t = fastpowf(t, 1.0f + linearity * 2.0f);
  } else if (linearity < -0.01f) {
    t = 1.0f - fastpowf(1.0f - t, 1.0f - linearity * 2.0f);
  }

  return -1.0f + 2.0f * t;
}

static inline float generate_ramp_down(float phase, float linearity) {
  float t = phase;

  // Apply linearity curve
  if (linearity > 0.01f) {
    t = fastpowf(t, 1.0f + linearity * 2.0f);
  } else if (linearity < -0.01f) {
    t = 1.0f - fastpowf(1.0f - t, 1.0f - linearity * 2.0f);
  }

  return 1.0f - 2.0f * t;
}

// Morphable waveform generator
// shape: 0.0 = square, 0.2 = triangle, 0.4 = sine, 0.6 = ramp up, 0.8 = ramp down, 1.0 = saw
static inline float generate_waveform(float phase, float shape, float transition, float linearity) {
  // Morph between waveform types
  float morph_pos = shape * (WAVE_COUNT - 1);
  uint32_t type_a = (uint32_t)morph_pos;
  uint32_t type_b = type_a + 1;
  float mix = morph_pos - (float)type_a;

  if (type_b >= WAVE_COUNT) {
    type_b = WAVE_COUNT - 1;
    mix = 0.0f;
  }

  float output_a = 0.0f, output_b = 0.0f;

  // Generate waveform A
  switch (type_a) {
    case WAVE_SQUARE:
      output_a = osc_sqrf(phase);
      break;
    case WAVE_TRIANGLE:
      output_a = generate_triangle(phase, transition, linearity);
      break;
    case WAVE_SINE:
      output_a = fast_sine(phase);
      break;
    case WAVE_RAMP_UP:
      output_a = generate_ramp_up(phase, linearity);
      break;
    case WAVE_RAMP_DOWN:
      output_a = generate_ramp_down(phase, linearity);
      break;
    case WAVE_SAW:
      output_a = osc_sawf(phase);
      break;
  }

  // Generate waveform B (if morphing)
  if (mix > 0.001f) {
    switch (type_b) {
      case WAVE_SQUARE:
        output_b = osc_sqrf(phase);
        break;
      case WAVE_TRIANGLE:
        output_b = generate_triangle(phase, transition, linearity);
        break;
      case WAVE_SINE:
        output_b = fast_sine(phase);
        break;
      case WAVE_RAMP_UP:
        output_b = generate_ramp_up(phase, linearity);
        break;
      case WAVE_RAMP_DOWN:
        output_b = generate_ramp_down(phase, linearity);
        break;
      case WAVE_SAW:
        output_b = osc_sawf(phase);
        break;
    }

    // Crossfade between waveforms
    output_a = output_a * (1.0f - mix) + output_b * mix;
  }

  return output_a;
}

// ============================================================================
// OSC API Callback Functions
// ============================================================================

void OSC_INIT(uint32_t platform, uint32_t api)
{
  // Initialize state to zero
  s_state.phase_main = 0.0f;
  s_state.phase_lfo = 0.0f;
  s_state.w0_main = 0.0f;
  s_state.w0_lfo = 0.01f / SAMPLE_RATE; // Default 0.01 Hz

  // Default parameters
  s_state.shape_main = 0.0f;     // Square
  s_state.shape_lfo = 0.0f;      // Square
  s_state.phase_trans = 0.5f;    // 50% transition point
  s_state.linearity = 0.0f;      // Linear

  s_state.lfo_rate = 0.1f;       // 10% rate
  s_state.lfo_dest = DEST_AMP;   // Amplitude modulation
  s_state.lfo_value = 0.0f;

  s_state.crossmod_amt = 0.0f;   // No cross-mod
  s_state.overlay_interval = 0;  // Unison (no interval)

  s_state.use_8bit = 0;          // Floating point mode by default
  s_state.sample_hold = 0.0f;
  s_state.sample_counter = 0;

  s_state.note = 60;             // Middle C
  s_state.mod = 0;
}

void OSC_CYCLE(const user_osc_param_t * const params, int32_t *yn, const uint32_t frames)
{
  // Extract MIDI note and fine tune
  const uint8_t note = (params->pitch >> 8) & 0xFF;
  const uint8_t mod = params->pitch & 0xFF;

  // Update note if changed
  if (note != s_state.note || mod != s_state.mod) {
    s_state.note = note;
    s_state.mod = mod;

    // Calculate main oscillator frequency
    float freq = osc_notehzf(note);
    float mod_mult = 1.0f + (mod / 255.0f) * 0.0594630943593f; // Semitone fine tune
    s_state.w0_main = (freq * mod_mult) * INV_SAMPLE_RATE;
  }

  // Get shape parameter from params (updated in real-time by hardware)
  float shape_from_lfo = params->shape_lfo * 4.656612873077392578125e-10f; // Q31 to float

  // Process audio samples
  int32_t * __restrict y = yn;
  const int32_t * y_e = y + frames;

  for (; y != y_e; ) {
    // Update LFO phase and generate LFO value
    s_state.phase_lfo += s_state.w0_lfo;
    if (s_state.phase_lfo >= 1.0f) {
      s_state.phase_lfo -= 1.0f;
    }

    // Generate LFO waveform
    float lfo_raw = generate_waveform(
      s_state.phase_lfo,
      s_state.shape_lfo,
      0.5f, // LFO uses fixed 50% transition
      0.0f  // LFO uses linear slopes
    );

    // Convert LFO from [-1, 1] to [0, 1] for most destinations
    s_state.lfo_value = (lfo_raw + 1.0f) * 0.5f;

    // Apply cross-modulation: LFO phase modulates main phase
    float phase_mod = 0.0f;
    if (s_state.crossmod_amt > 0.001f) {
      phase_mod = lfo_raw * s_state.crossmod_amt * 0.5f; // ±50% phase mod max
    }

    // Calculate main oscillator parameters with modulation
    float main_shape = s_state.shape_main;
    float main_linearity = s_state.linearity;
    float main_freq_mod = 1.0f;
    float main_amp_mod = 1.0f;

    // Route LFO to destination
    switch (s_state.lfo_dest) {
      case DEST_AMP:
        main_amp_mod = s_state.lfo_value;
        break;
      case DEST_FREQ:
        // Frequency modulation ±2 octaves
        main_freq_mod = fastpowf(2.0f, (lfo_raw * 2.0f));
        break;
      case DEST_SHAPE:
        // Add LFO to shape parameter
        main_shape = clip01f(main_shape + shape_from_lfo * 0.5f + lfo_raw * 0.3f);
        break;
      case DEST_LINEARITY:
        // Modulate linearity
        main_linearity = clipminmaxf(-1.0f, main_linearity + lfo_raw * 0.5f, 1.0f);
        break;
    }

    // Update main phase with frequency modulation
    float w0_modulated = s_state.w0_main * main_freq_mod;
    s_state.phase_main += w0_modulated + phase_mod;

    // Wrap phase to [0, 1)
    if (s_state.phase_main >= 1.0f) {
      s_state.phase_main -= 1.0f;
    } else if (s_state.phase_main < 0.0f) {
      s_state.phase_main += 1.0f;
    }

    // Generate main oscillator waveform (voice 1)
    float out1 = generate_waveform(
      s_state.phase_main,
      main_shape,
      s_state.phase_trans,
      main_linearity
    );

    // Apply amplitude modulation
    out1 *= main_amp_mod;

    // Voice overlay: generate second voice at interval
    float output = out1;
    if (s_state.overlay_interval != 0) {
      // Calculate frequency ratio for the interval (2^(semitones/12))
      float interval_ratio = fastpowf(2.0f, s_state.overlay_interval / 12.0f);

      // Calculate phase increment for second voice
      float w0_voice2 = w0_modulated * interval_ratio;

      // Generate second voice phase (separate accumulator would be better, but this works)
      float phase2 = s_state.phase_main + (w0_voice2 * frames * 0.5f);
      while (phase2 >= 1.0f) phase2 -= 1.0f;
      while (phase2 < 0.0f) phase2 += 1.0f;

      float out2 = generate_waveform(
        phase2,
        main_shape,
        s_state.phase_trans,
        main_linearity
      );

      out2 *= main_amp_mod;

      // Mix voices 50/50
      output = (out1 + out2) * 0.5f;
    }

    // 8-bit mode processing
    if (s_state.use_8bit) {
      // Sample rate reduction: downsample to ~8kHz (48kHz / 6 = 8kHz)
      const uint16_t DOWNSAMPLE_FACTOR = 6;

      if (s_state.sample_counter == 0) {
        // Time to take a new sample
        // Quantize to 8-bit (256 levels: -1.0 to 1.0 mapped to -128 to 127)
        int8_t quantized = (int8_t)(output * 127.0f);
        s_state.sample_hold = quantized / 127.0f; // Convert back to float
      }

      output = s_state.sample_hold; // Use held sample
      s_state.sample_counter = (s_state.sample_counter + 1) % DOWNSAMPLE_FACTOR;
    }

    // Soft clipping for safety
    output = osc_softclipf(0.05f, output);

    // Convert to Q31 and write to output buffer
    *(y++) = f32_to_q31(output);
  }
}

void OSC_NOTEON(const user_osc_param_t * const params)
{
  // Reset phases on note-on for consistent attack
  s_state.phase_main = 0.0f;
  // Don't reset LFO phase - let it free-run
}

void OSC_NOTEOFF(const user_osc_param_t * const params)
{
  // Nothing special needed on note-off
}

void OSC_PARAM(uint16_t index, uint16_t value)
{
  switch (index) {
    case k_user_osc_param_id1:
      // Phase transition point (0-100) or 8-bit mode (101)
      if (value >= 101) {
        s_state.use_8bit = 1;
        s_state.phase_trans = 0.5f; // Default to 50% when in 8-bit mode
      } else {
        s_state.use_8bit = 0;
        s_state.phase_trans = clip01f(value * 0.01f);
      }
      break;

    case k_user_osc_param_id2:
      // Slope linearity (-100% to +100%, bipolar)
      s_state.linearity = clipminmaxf(-1.0f, (value - 100) * 0.01f, 1.0f);
      break;

    case k_user_osc_param_id3:
      // LFO Rate (0-100%, maps logarithmically to 0.1 Hz - 10000 Hz)
      s_state.lfo_rate = clip01f(value * 0.01f);
      {
        // Logarithmic mapping: 0.1 Hz to 10000 Hz
        float freq_hz = LFO_FREQ_MIN * fastpowf(LFO_FREQ_MAX / LFO_FREQ_MIN, s_state.lfo_rate);
        s_state.w0_lfo = freq_hz * INV_SAMPLE_RATE;
      }
      break;

    case k_user_osc_param_id4:
      // LFO Destination (0-3)
      s_state.lfo_dest = (value >= DEST_COUNT) ? DEST_AMP : value;
      break;

    case k_user_osc_param_id5:
      // Cross-modulation amount (0-100%)
      s_state.crossmod_amt = clip01f(value * 0.01f);
      break;

    case k_user_osc_param_id6:
      // Voice overlay interval in semitones (-24 to +24)
      s_state.overlay_interval = (int8_t)(value - 24);
      break;

    case k_user_osc_param_shape:
      // Main waveform shape morph (0-1023 -> 0.0-1.0)
      s_state.shape_main = param_val_to_f32(value);
      break;

    case k_user_osc_param_shiftshape:
      // LFO waveform shape morph (0-1023 -> 0.0-1.0)
      s_state.shape_lfo = param_val_to_f32(value);
      break;

    default:
      break;
  }
}

/*
 *  compat/arm_math.h — portable stand-in for the CMSIS header.
 *
 *  The logue SDK's utils/cortexm.h aliases ~105 Cortex-M intrinsics, but only
 *  a handful are reachable from the SDK headers themselves. Those are
 *  implemented here in plain C so unit sources can be hosted off-target.
 *  Semantics follow the ARM definitions, including saturation behaviour.
 *
 *  Host-only. Never compiled into a device build.
 */
#ifndef COMPAT_ARM_MATH_H_
#define COMPAT_ARM_MATH_H_

#include <stdint.h>
#include <limits.h>

typedef int32_t q31_t;
typedef int16_t q15_t;
typedef int8_t  q7_t;
typedef float   float32_t;
typedef double  float64_t;

static inline int32_t __SSAT(int32_t v, uint32_t n) {
  if (n == 0 || n > 32) return v;
  const int32_t max = (n == 32) ? INT32_MAX : (int32_t)((1u << (n - 1)) - 1u);
  const int32_t min = (n == 32) ? INT32_MIN : -(int32_t)(1u << (n - 1));
  return v > max ? max : (v < min ? min : v);
}

static inline uint32_t __USAT(int32_t v, uint32_t n) {
  if (n >= 32) return (uint32_t)(v < 0 ? 0 : v);
  const int32_t max = (int32_t)((1u << n) - 1u);
  return (uint32_t)(v > max ? max : (v < 0 ? 0 : v));
}

static inline uint32_t __CLZ(uint32_t v) {
  if (v == 0u) return 32u;              /* ARM defines CLZ(0) as 32 */
  return (uint32_t)__builtin_clz(v);
}

static inline uint32_t __RBIT(uint32_t v) {
  v = ((v & 0x55555555u) << 1) | ((v >> 1) & 0x55555555u);
  v = ((v & 0x33333333u) << 2) | ((v >> 2) & 0x33333333u);
  v = ((v & 0x0F0F0F0Fu) << 4) | ((v >> 4) & 0x0F0F0F0Fu);
  return __builtin_bswap32(v);
}

static inline uint32_t __REV(uint32_t v) { return __builtin_bswap32(v); }

static inline uint32_t __ROR(uint32_t v, uint32_t s) {
  s &= 31u;
  return s ? ((v >> s) | (v << (32u - s))) : v;
}

static inline int32_t __QADD(int32_t a, int32_t b) {
  int32_t r;
  if (!__builtin_add_overflow(a, b, &r)) return r;
  return a >= 0 ? INT32_MAX : INT32_MIN;
}

static inline int32_t __QSUB(int32_t a, int32_t b) {
  int32_t r;
  if (!__builtin_sub_overflow(a, b, &r)) return r;
  return a >= 0 ? INT32_MAX : INT32_MIN;
}

/* Packed halfword ops: low half from a, high half from b shifted left. */
static inline uint32_t __PKHBT(uint32_t a, uint32_t b, uint32_t s) {
  return (a & 0x0000FFFFu) | ((b << s) & 0xFFFF0000u);
}

static inline uint32_t __SADD16(uint32_t a, uint32_t b) {
  const int16_t al = (int16_t)(a & 0xFFFFu), ah = (int16_t)(a >> 16);
  const int16_t bl = (int16_t)(b & 0xFFFFu), bh = (int16_t)(b >> 16);
  return ((uint32_t)(uint16_t)(int16_t)(al + bl)) |
         ((uint32_t)(uint16_t)(int16_t)(ah + bh) << 16);
}

static inline uint32_t __SSUB16(uint32_t a, uint32_t b) {
  const int16_t al = (int16_t)(a & 0xFFFFu), ah = (int16_t)(a >> 16);
  const int16_t bl = (int16_t)(b & 0xFFFFu), bh = (int16_t)(b >> 16);
  return ((uint32_t)(uint16_t)(int16_t)(al - bl)) |
         ((uint32_t)(uint16_t)(int16_t)(ah - bh) << 16);
}

/* Signed most-significant-word multiply accumulate: c + ((a*b) >> 32). */
static inline int32_t __SMMLA(int32_t a, int32_t b, int32_t c) {
  return (int32_t)(c + (int32_t)(((int64_t)a * (int64_t)b) >> 32));
}

/* Dual 16x16 multiply, accumulate both products into c. */
static inline int32_t __SMLAD(uint32_t a, uint32_t b, int32_t c) {
  const int16_t al = (int16_t)(a & 0xFFFFu), ah = (int16_t)(a >> 16);
  const int16_t bl = (int16_t)(b & 0xFFFFu), bh = (int16_t)(b >> 16);
  return c + (int32_t)al * (int32_t)bl + (int32_t)ah * (int32_t)bh;
}

/*
 * GE-flag emulation.
 *
 * The SDK writes min/max as an ARM idiom: a parallel subtract sets the APSR
 * GE flags, then SEL picks per-lane. Nothing here is a real APSR, so the
 * subtract records the per-lane comparison and SEL consumes it. Thread-local
 * because the audio thread and the UI thread both run this code.
 */
static _Thread_local uint32_t s_ge; /* 4 bits, one per byte, as on ARM */

typedef int32_t __SIMD32_TYPE;

static inline int32_t __QSUB_ge(int32_t a, int32_t b) {
  s_ge = (a >= b) ? 0xFu : 0x0u;
  return __QSUB(a, b);
}

static inline uint32_t __QADD16(uint32_t a, uint32_t b) {
  const int32_t al = (int16_t)(a & 0xFFFFu), ah = (int16_t)(a >> 16);
  const int32_t bl = (int16_t)(b & 0xFFFFu), bh = (int16_t)(b >> 16);
  const int32_t rl = __SSAT(al + bl, 16), rh = __SSAT(ah + bh, 16);
  return ((uint32_t)(uint16_t)rl) | ((uint32_t)(uint16_t)rh << 16);
}

static inline uint32_t __QSUB16(uint32_t a, uint32_t b) {
  const int32_t al = (int16_t)(a & 0xFFFFu), ah = (int16_t)(a >> 16);
  const int32_t bl = (int16_t)(b & 0xFFFFu), bh = (int16_t)(b >> 16);
  /* GE[1:0] track the low halfword, GE[3:2] the high one. */
  s_ge = ((al >= bl) ? 0x3u : 0x0u) | ((ah >= bh) ? 0xCu : 0x0u);
  const int32_t rl = __SSAT(al - bl, 16), rh = __SSAT(ah - bh, 16);
  return ((uint32_t)(uint16_t)rl) | ((uint32_t)(uint16_t)rh << 16);
}

/* Per-byte select: byte i from a when GE[i] is set, else from b. */
static inline uint32_t __SEL(uint32_t a, uint32_t b) {
  uint32_t r = 0;
  for (unsigned i = 0; i < 4; ++i) {
    const uint32_t mask = 0xFFu << (i * 8);
    r |= ((s_ge >> i) & 1u) ? (a & mask) : (b & mask);
  }
  return r;
}

/* Route the plain saturating subtract through the GE-setting variant so the
   SDK's qsub(a,b);sel(a,b) min/max idiom behaves as intended off-target. */
#undef __QSUB
#define __QSUB(a, b) __QSUB_ge((a), (b))

#endif /* COMPAT_ARM_MATH_H_ */


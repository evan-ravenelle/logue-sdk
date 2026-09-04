/*
 *  cdhost.h — normalized host ABI for logue-sdk units.
 *
 *  Platforms do not share binary layouts: drumlogue and nts-3 disagree on
 *  the width of unit_header.target, on UNIT_MAX_PARAM_COUNT (24 vs 8), on
 *  name lengths (13/12 vs 19/21), on struct packing, and on the shape of
 *  unit_runtime_desc_t. So each platform gets its own shim compiled against
 *  its own SDK headers, and they all present the interface below. Nothing
 *  here hand-encodes an offset.
 */
#ifndef CDHOST_H_
#define CDHOST_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  CDH_PLATFORM_DRUMLOGUE = 0,
  CDH_PLATFORM_NTS3 = 1,
  CDH_NUM_PLATFORMS
};

/* Touch states, mirroring the nts-3 runtime. */
enum { CDH_TOUCH_RELEASED = 0, CDH_TOUCH_PRESSED = 1, CDH_TOUCH_MOVED = 2 };

/* Widest common denominator of every platform's unit_param_t. */
typedef struct {
  int16_t min, max, center, init;
  uint8_t type, frac, frac_mode;
  char name[32];
} cdh_param;

/* One loaded unit. Opaque; owned by the platform shim. */
typedef struct cdh_unit cdh_unit;

/*
 * Function table a platform shim fills in. `self` is the shim's own handle.
 * Optional entries may be NULL and are feature-detected by the engine.
 */
typedef struct {
  const char *platform_name;
  void *(*open)(const char *dylib_path, uint32_t samplerate, uint16_t frames,
                char *err, int errlen);
  void (*close)(void *self);
  const char *(*name)(void *self);
  int (*param_count)(void *self);
  int (*param_get)(void *self, int idx, cdh_param *out);
  void (*set_param)(void *self, int id, int32_t value);
  int32_t (*get_param)(void *self, int id);
  const uint8_t *(*get_bitmap)(void *self, int id);   /* NULL if unsupported */
  void (*render)(void *self, const float *in, float *out, uint32_t frames);
  void (*reset)(void *self);
  int (*in_channels)(void *self);
  int (*out_channels)(void *self);
  void (*touch)(void *self, int id, float x, float y, int state); /* NULL if none */
  void (*set_tempo)(void *self, float bpm);                       /* NULL if none */
  /* Optional per-unit diagnostics, feature-detected. A unit opts in by
     exporting unit_get_diag_count / unit_get_diag; most do not. */
  int (*diag_count)(void *self);
  const char *(*diag_get)(void *self, int idx, float *value);
} cdh_vtable;

extern const cdh_vtable cdh_vtable_drumlogue;
extern const cdh_vtable cdh_vtable_nts3;

#ifdef __cplusplus
}
#endif
#endif /* CDHOST_H_ */

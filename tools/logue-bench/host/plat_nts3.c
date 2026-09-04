/*
 *  NTS-3 kaoss platform shim. Compiled against platform/nts-3_kaoss/common.
 *
 *  Same unit-API family as the drumlogue, but not binary compatible with it:
 *  32-bit target, 8 params instead of 24, longer names, packed structs, and a
 *  runtime descriptor built around a hooks struct with SDRAM callbacks and a
 *  module-specific context. Hence a separate shim over the same interface.
 */
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "runtime.h"
#include "unit_genericfx.h"
#include "cdhost.h"

/* The Kaoss pad's logical touch area. The unit reads this from the context. */
#define NTS3_TOUCH_W 1024u
#define NTS3_TOUCH_H 1024u

/* Stand-in SDRAM arena. The device gives units a dedicated region; here a
   plain heap is indistinguishable from the unit's point of view. */
static uint8_t *sdram_alloc(size_t size) { return (uint8_t *)malloc(size); }
static void sdram_free(const uint8_t *p) { free((void *)p); }
static size_t sdram_avail(void) { return 1u << 22; } /* 4 MB, arbitrary */

typedef struct {
  void *dl;
  const genericfx_unit_header_t *hdr;
  unit_runtime_desc_t desc;
  unit_runtime_genericfx_context_t ctx;
  const float *raw_input;   /* refreshed each render; see get_raw_input */

  int8_t (*init)(const unit_runtime_desc_t *);
  void (*teardown)(void);
  void (*reset)(void);
  void (*resume)(void);
  void (*suspend)(void);
  void (*render)(const float *, float *, uint32_t);
  void (*set_param)(uint8_t, int32_t);
  int32_t (*get_param)(uint8_t);
  void (*set_tempo)(uint32_t);
  int32_t (*diag_count)(void);
  const char *(*diag_get)(int32_t, float *);
  void (*touch_event)(uint8_t, uint8_t, uint32_t, uint32_t);
} n3_unit;

/* Only one unit renders at a time per shim instance; the runtime contract is
   that get_raw_input() is valid only inside render(). */
static const float *g_raw_input;
static const float *get_raw_input(void) { return g_raw_input; }

static void *n3_open(const char *path, uint32_t sr, uint16_t frames, char *err, int errlen) {
  n3_unit *u = calloc(1, sizeof(n3_unit));
  if (!u) return 0;

  u->dl = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (!u->dl) { snprintf(err, errlen, "dlopen: %s", dlerror()); free(u); return 0; }

  u->hdr = (const genericfx_unit_header_t *)dlsym(u->dl, "unit_header");
  u->init = (int8_t (*)(const unit_runtime_desc_t *))dlsym(u->dl, "unit_init");
  u->render = (void (*)(const float *, float *, uint32_t))dlsym(u->dl, "unit_render");
  if (!u->hdr || !u->init || !u->render) {
    snprintf(err, errlen, "not an nts-3 unit (missing unit_header/init/render)");
    dlclose(u->dl); free(u); return 0;
  }
  u->teardown = (void (*)(void))dlsym(u->dl, "unit_teardown");
  u->reset = (void (*)(void))dlsym(u->dl, "unit_reset");
  u->resume = (void (*)(void))dlsym(u->dl, "unit_resume");
  u->suspend = (void (*)(void))dlsym(u->dl, "unit_suspend");
  u->set_param = (void (*)(uint8_t, int32_t))dlsym(u->dl, "unit_set_param_value");
  u->get_param = (int32_t (*)(uint8_t))dlsym(u->dl, "unit_get_param_value");
  u->set_tempo = (void (*)(uint32_t))dlsym(u->dl, "unit_set_tempo");
  u->diag_count = (int32_t (*)(void))dlsym(u->dl, "unit_get_diag_count");
  u->diag_get = (const char *(*)(int32_t, float *))dlsym(u->dl, "unit_get_diag");
  u->touch_event = (void (*)(uint8_t, uint8_t, uint32_t, uint32_t))dlsym(u->dl, "unit_touch_event");

  memset(&u->ctx, 0, sizeof(u->ctx));
  u->ctx.touch_area_width = NTS3_TOUCH_W;
  u->ctx.touch_area_height = NTS3_TOUCH_H;
  u->ctx.get_raw_input = get_raw_input;

  memset(&u->desc, 0, sizeof(u->desc));
  u->desc.target = u->hdr->common.target;
  u->desc.api = u->hdr->common.api;
  u->desc.samplerate = sr;
  u->desc.frames_per_buffer = frames;
  u->desc.input_channels = 2;
  u->desc.output_channels = 2;
  u->desc.hooks.runtime_context = (const unit_runtime_base_context_t *)&u->ctx;
  u->desc.hooks.sdram_alloc = sdram_alloc;
  u->desc.hooks.sdram_free = sdram_free;
  u->desc.hooks.sdram_avail = sdram_avail;

  int8_t rc = u->init(&u->desc);
  if (rc != k_unit_err_none) {
    snprintf(err, errlen, "unit_init failed (%d)", (int)rc);
    dlclose(u->dl); free(u); return 0;
  }
  if (u->set_param)
    for (uint32_t i = 0; i < u->hdr->common.num_params; ++i)
      u->set_param((uint8_t)i, u->hdr->common.params[i].init);
  if (u->reset) u->reset();
  if (u->resume) u->resume();
  return u;
}

static void n3_close(void *self) {
  n3_unit *u = self; if (!u) return;
  if (u->suspend) u->suspend();
  if (u->teardown) u->teardown();
  if (u->dl) dlclose(u->dl);
  free(u);
}

static const char *n3_name(void *self) { return ((n3_unit *)self)->hdr->common.name; }
static int n3_pcount(void *self) { return (int)((n3_unit *)self)->hdr->common.num_params; }

static int n3_pget(void *self, int idx, cdh_param *o) {
  n3_unit *u = self;
  if (idx < 0 || idx >= (int)u->hdr->common.num_params) return -1;
  const unit_param_t *p = &u->hdr->common.params[idx];
  o->min = p->min; o->max = p->max; o->center = p->center; o->init = p->init;
  o->type = p->type; o->frac = p->frac; o->frac_mode = p->frac_mode;
  memset(o->name, 0, sizeof(o->name));
  memcpy(o->name, p->name, UNIT_PARAM_NAME_LEN);
  return 0;
}

static void n3_set(void *self, int id, int32_t v) {
  n3_unit *u = self; if (u->set_param) u->set_param((uint8_t)id, v);
}
static int32_t n3_get(void *self, int id) {
  n3_unit *u = self; return u->get_param ? u->get_param((uint8_t)id) : 0;
}
static void n3_render(void *self, const float *in, float *out, uint32_t n) {
  n3_unit *u = self;
  g_raw_input = in;              /* valid only for the duration of render() */
  u->render(in, out, n);
  g_raw_input = 0;
}
static void n3_reset(void *self) { n3_unit *u = self; if (u->reset) u->reset(); }
static int n3_inch(void *self) { return ((n3_unit *)self)->desc.input_channels; }
static int n3_outch(void *self) { return ((n3_unit *)self)->desc.output_channels; }

/* x/y arrive normalised 0..1 from the UI; the unit expects touch-area units. */
static void n3_touch(void *self, int id, float x, float y, int state) {
  n3_unit *u = self;
  if (!u->touch_event) return;
  if (x < 0.f) x = 0.f; else if (x > 1.f) x = 1.f;
  if (y < 0.f) y = 0.f; else if (y > 1.f) y = 1.f;
  u->touch_event((uint8_t)id, (uint8_t)state,
                 (uint32_t)(x * (NTS3_TOUCH_W - 1)),
                 (uint32_t)(y * (NTS3_TOUCH_H - 1)));
}

static void n3_tempo(void *self, float bpm) {
  n3_unit *u = self;
  if (!u->set_tempo) return;
  uint32_t i = (uint32_t)bpm;
  uint32_t f = (uint32_t)((bpm - (float)i) * 65536.0f);
  u->set_tempo((i << 16) | (f & 0xFFFF));
}

static int n3_diag_count(void *self) {
  n3_unit *u = self; return u->diag_count ? (int)u->diag_count() : 0;
}
static const char *n3_diag_get(void *self, int idx, float *value) {
  n3_unit *u = self; return u->diag_get ? u->diag_get((int32_t)idx, value) : 0;
}

const cdh_vtable cdh_vtable_nts3 = {
    "nts-3_kaoss", n3_open, n3_close, n3_name, n3_pcount, n3_pget, n3_set,
    n3_get, 0 /* no bitmap params */, n3_render, n3_reset, n3_inch, n3_outch,
    n3_touch, n3_tempo, n3_diag_count, n3_diag_get};

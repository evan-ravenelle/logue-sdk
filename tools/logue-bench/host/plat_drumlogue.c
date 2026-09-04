/*
 *  Drumlogue platform shim. Compiled against platform/drumlogue/common.
 */
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "runtime.h"
#include "cdhost.h"

typedef struct {
  void *dl;
  const unit_header_t *hdr;
  unit_runtime_desc_t desc;

  int8_t (*init)(const unit_runtime_desc_t *);
  void (*teardown)(void);
  void (*reset)(void);
  void (*resume)(void);
  void (*suspend)(void);
  void (*render)(const float *, float *, uint32_t);
  void (*set_param)(uint8_t, int32_t);
  int32_t (*get_param)(uint8_t);
  const uint8_t *(*get_bmp)(uint8_t, int32_t);
  void (*set_tempo)(uint32_t);
  int32_t (*diag_count)(void);
  const char *(*diag_get)(int32_t, float *);
} dl_unit;

static uint8_t no_banks(void) { return 0; }
static uint8_t no_samples(uint8_t b) { (void)b; return 0; }
static const sample_wrapper_t *no_sample(uint8_t b, uint8_t s) { (void)b; (void)s; return 0; }

static void *dl_open(const char *path, uint32_t sr, uint16_t frames, char *err, int errlen) {
  dl_unit *u = calloc(1, sizeof(dl_unit));
  if (!u) return 0;

  u->dl = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (!u->dl) { snprintf(err, errlen, "dlopen: %s", dlerror()); free(u); return 0; }

  u->hdr = (const unit_header_t *)dlsym(u->dl, "unit_header");
  u->init = (int8_t (*)(const unit_runtime_desc_t *))dlsym(u->dl, "unit_init");
  u->render = (void (*)(const float *, float *, uint32_t))dlsym(u->dl, "unit_render");
  if (!u->hdr || !u->init || !u->render) {
    snprintf(err, errlen, "not a drumlogue unit (missing unit_header/init/render)");
    dlclose(u->dl); free(u); return 0;
  }
  u->teardown = (void (*)(void))dlsym(u->dl, "unit_teardown");
  u->reset = (void (*)(void))dlsym(u->dl, "unit_reset");
  u->resume = (void (*)(void))dlsym(u->dl, "unit_resume");
  u->suspend = (void (*)(void))dlsym(u->dl, "unit_suspend");
  u->set_param = (void (*)(uint8_t, int32_t))dlsym(u->dl, "unit_set_param_value");
  u->get_param = (int32_t (*)(uint8_t))dlsym(u->dl, "unit_get_param_value");
  u->get_bmp = (const uint8_t *(*)(uint8_t, int32_t))dlsym(u->dl, "unit_get_param_bmp_value");
  u->set_tempo = (void (*)(uint32_t))dlsym(u->dl, "unit_set_tempo");
  u->diag_count = (int32_t (*)(void))dlsym(u->dl, "unit_get_diag_count");
  u->diag_get = (const char *(*)(int32_t, float *))dlsym(u->dl, "unit_get_diag");

  memset(&u->desc, 0, sizeof(u->desc));
  /* Take target/api from the unit itself so its own compatibility checks pass. */
  u->desc.target = u->hdr->target;
  u->desc.api = u->hdr->api;
  u->desc.samplerate = sr;
  u->desc.frames_per_buffer = frames;
  /* Masterfx takes main L/R plus sidechain L/R; other modules take stereo. */
  u->desc.input_channels =
      ((u->hdr->target & 0xFF) == k_unit_module_masterfx) ? 4 : 2;
  u->desc.output_channels = 2;
  u->desc.get_num_sample_banks = no_banks;
  u->desc.get_num_samples_for_bank = no_samples;
  u->desc.get_sample = no_sample;

  int8_t rc = u->init(&u->desc);
  if (rc != k_unit_err_none) {
    snprintf(err, errlen, "unit_init failed (%d)", (int)rc);
    dlclose(u->dl); free(u); return 0;
  }
  if (u->set_param)
    for (uint32_t i = 0; i < u->hdr->num_params; ++i)
      u->set_param((uint8_t)i, u->hdr->params[i].init);
  if (u->reset) u->reset();
  if (u->resume) u->resume();
  return u;
}

static void dl_close(void *self) {
  dl_unit *u = self; if (!u) return;
  if (u->suspend) u->suspend();
  if (u->teardown) u->teardown();
  if (u->dl) dlclose(u->dl);
  free(u);
}

static const char *dl_name(void *self) { return ((dl_unit *)self)->hdr->name; }
static int dl_pcount(void *self) { return (int)((dl_unit *)self)->hdr->num_params; }

static int dl_pget(void *self, int idx, cdh_param *o) {
  dl_unit *u = self;
  if (idx < 0 || idx >= (int)u->hdr->num_params) return -1;
  const unit_param_t *p = &u->hdr->params[idx];
  o->min = p->min; o->max = p->max; o->center = p->center; o->init = p->init;
  o->type = p->type; o->frac = p->frac; o->frac_mode = p->frac_mode;
  memset(o->name, 0, sizeof(o->name));
  memcpy(o->name, p->name, UNIT_PARAM_NAME_LEN);
  return 0;
}

static void dl_set(void *self, int id, int32_t v) {
  dl_unit *u = self; if (u->set_param) u->set_param((uint8_t)id, v);
}
static int32_t dl_get(void *self, int id) {
  dl_unit *u = self; return u->get_param ? u->get_param((uint8_t)id) : 0;
}
static const uint8_t *dl_bmp(void *self, int id) {
  dl_unit *u = self; return u->get_bmp ? u->get_bmp((uint8_t)id, 0) : 0;
}
static void dl_render(void *self, const float *in, float *out, uint32_t n) {
  ((dl_unit *)self)->render(in, out, n);
}
static void dl_reset(void *self) { dl_unit *u = self; if (u->reset) u->reset(); }
static int dl_inch(void *self) { return ((dl_unit *)self)->desc.input_channels; }
static int dl_outch(void *self) { return ((dl_unit *)self)->desc.output_channels; }
static void dl_tempo(void *self, float bpm) {
  dl_unit *u = self;
  if (!u->set_tempo) return;
  uint32_t i = (uint32_t)bpm;
  uint32_t f = (uint32_t)((bpm - (float)i) * 65536.0f);
  u->set_tempo((i << 16) | (f & 0xFFFF));
}

static int dl_diag_count(void *self) {
  dl_unit *u = self; return u->diag_count ? (int)u->diag_count() : 0;
}
static const char *dl_diag_get(void *self, int idx, float *value) {
  dl_unit *u = self; return u->diag_get ? u->diag_get((int32_t)idx, value) : 0;
}

const cdh_vtable cdh_vtable_drumlogue = {
    "drumlogue", dl_open, dl_close, dl_name, dl_pcount, dl_pget, dl_set,
    dl_get, dl_bmp, dl_render, dl_reset, dl_inch, dl_outch,
    0 /* no touch */, dl_tempo, dl_diag_count, dl_diag_get};

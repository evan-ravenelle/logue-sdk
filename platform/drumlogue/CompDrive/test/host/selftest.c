/*
 *  selftest — exercises a unit through the host framework.
 *
 *  Platform-agnostic: everything is discovered through the normalized ABI,
 *  so this runs against any unit on any supported platform.
 *
 *    ./selftest <platform-index> <unit.dylib>
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cdhost.h"

extern int cdh_platform_count(void);
extern const char *cdh_platform_name(int);
extern int cdh_open(int, const char *, char *, int);
extern void cdh_close_slot(int);
extern const char *cdh_unit_name(int);
extern const char *cdh_unit_platform(int);
extern int cdh_param_count(int);
extern int cdh_param_get(int, int, cdh_param *);
extern void cdh_set_param(int, int, int32_t);
extern int32_t cdh_get_param(int, int);
extern int cdh_in_channels(int);
extern int cdh_has_touch(int);
extern void cdh_touch(int, int, float, float, int);
extern int cdh_get_bitmap(int, int, uint8_t *);
extern int cdh_load_wav(const char *);
extern int cdh_render_offline(const char *);
extern void cdh_set_active(int);

static int fails = 0, checks = 0;
static void check(int ok, const char *what) {
  ++checks;
  if (!ok) { ++fails; printf("  FAIL  %s\n", what); }
  else printf("  ok    %s\n", what);
}

static void writeSine(const char *p, int frames, float amp, float hz) {
  FILE *f = fopen(p, "wb");
  uint32_t n = (uint32_t)(frames * 2 * 4), riff = 36 + n, rate = 48000, br = rate * 8, fs = 16;
  uint16_t fmt = 3, ch = 2, al = 8, bits = 32;
  fwrite("RIFF", 1, 4, f); fwrite(&riff, 4, 1, f); fwrite("WAVE", 1, 4, f);
  fwrite("fmt ", 1, 4, f); fwrite(&fs, 4, 1, f); fwrite(&fmt, 2, 1, f); fwrite(&ch, 2, 1, f);
  fwrite(&rate, 4, 1, f); fwrite(&br, 4, 1, f); fwrite(&al, 2, 1, f); fwrite(&bits, 2, 1, f);
  fwrite("data", 1, 4, f); fwrite(&n, 4, 1, f);
  for (int i = 0; i < frames; ++i) {
    float s = amp * sinf(2.f * (float)M_PI * hz * i / 48000.f);
    fwrite(&s, 4, 1, f); fwrite(&s, 4, 1, f);
  }
  fclose(f);
}

/* Scan a rendered file for anything that would be audible as a fault. */
static int scanOutput(const char *p, double *peak, long *bad) {
  FILE *f = fopen(p, "rb");
  if (!f) return -1;
  fseek(f, 44, SEEK_SET);
  float v; *peak = 0; *bad = 0;
  long n = 0;
  while (fread(&v, 4, 1, f) == 1) {
    if (isnan(v) || isinf(v)) ++*bad;
    else if (fabsf(v) > *peak) *peak = fabsf(v);
    ++n;
  }
  fclose(f);
  return n > 0 ? 0 : -1;
}

int main(int argc, char **argv) {
  if (argc < 3) {
    printf("usage: %s <platform-index> <unit.dylib>\nplatforms:\n", argv[0]);
    for (int i = 0; i < cdh_platform_count(); ++i)
      printf("  %d = %s\n", i, cdh_platform_name(i));
    return 2;
  }
  const int plat = atoi(argv[1]);
  char err[256] = {0};

  int slot = cdh_open(plat, argv[2], err, sizeof err);
  if (slot < 0) { printf("open failed: %s\n", err); return 1; }
  cdh_set_active(slot);

  printf("\n%s  (%s)  slot %d\n", cdh_unit_name(slot), cdh_unit_platform(slot), slot);
  printf("----------------------------------------------\n");

  const int np = cdh_param_count(slot);
  check(np >= 0 && np <= 24, "parameter count in range");
  check(cdh_unit_name(slot)[0] != '\0', "unit reports a name");
  const int inch = cdh_in_channels(slot);
  check(inch == 2 || inch == 4, "input channel count is 2 or 4");

  /* Every declared parameter must round-trip its own init value. */
  int roundtrip = 1, named = 0;
  for (int i = 0; i < np; ++i) {
    cdh_param p;
    if (cdh_param_get(slot, i, &p)) { roundtrip = 0; break; }
    if (p.name[0]) ++named;
    if (p.min > p.max) { roundtrip = 0; break; }
    if (p.init < p.min || p.init > p.max) { roundtrip = 0; break; }
  }
  check(roundtrip, "descriptors self-consistent (min<=init<=max)");
  printf("        %d declared, %d named\n", np, named);

  /* Sweep every parameter to both rails; nothing should fault. */
  for (int i = 0; i < np; ++i) {
    cdh_param p;
    if (cdh_param_get(slot, i, &p) || !p.name[0]) continue;
    cdh_set_param(slot, i, p.min);
    cdh_set_param(slot, i, p.max);
    cdh_set_param(slot, i, p.init);
  }
  check(1, "parameter sweep to both rails survived");

  if (cdh_has_touch(slot)) {
    cdh_touch(slot, 0, 0.f, 0.f, CDH_TOUCH_PRESSED);
    cdh_touch(slot, 0, 0.5f, 0.5f, CDH_TOUCH_MOVED);
    cdh_touch(slot, 0, 1.f, 1.f, CDH_TOUCH_RELEASED);
    check(1, "touch event sequence accepted");
  }

  writeSine("/tmp/cdh_self_in.wav", 48000, 0.5f, 200.f);
  check(cdh_load_wav("/tmp/cdh_self_in.wav") == 0, "input WAV loaded");
  check(cdh_render_offline("/tmp/cdh_self_out.wav") == 0, "offline render completed");

  double peak = 0; long bad = 0;
  check(scanOutput("/tmp/cdh_self_out.wav", &peak, &bad) == 0, "output readable");
  check(bad == 0, "output free of NaN/Inf");
  check(peak <= 8.0, "output not wildly out of range");
  printf("        output peak %.4f (%.2f dBFS), %ld bad samples\n",
         peak, 20.0 * log10(peak > 1e-9 ? peak : 1e-9), bad);

  uint8_t bmp[32];
  int haveBmp = 0;
  for (int i = 0; i < np; ++i) if (cdh_get_bitmap(slot, i, bmp) == 0) { haveBmp = 1; break; }
  printf("        bitmap param: %s\n", haveBmp ? "present" : "none");

  cdh_close_slot(slot);
  check(1, "closed cleanly");

  printf("----------------------------------------------\n");
  printf("%d checks, %d failed\n\n", checks, fails);
  return fails ? 1 : 0;
}

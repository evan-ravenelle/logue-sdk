/*
 *  engine.cc — platform-agnostic host engine.
 *
 *  Owns the audio thread, transport, WAV I/O and meters. Knows nothing about
 *  any particular platform: everything unit-shaped goes through cdh_vtable.
 */
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <AudioToolbox/AudioToolbox.h>

#include "cdhost.h"

namespace {

constexpr uint32_t kSampleRate = 48000;
constexpr uint16_t kFramesPerBuffer = 64;
constexpr int kQueueBuffers = 3;
constexpr uint32_t kQueueFrames = 512;
constexpr uint32_t kMaxChannels = 4;

constexpr int kMaxUnits = 8;

/* One loaded unit. Slots are independent: each keeps its own parameters,
   so the UI can hold several open at once. */
struct Slot {
  const cdh_vtable *vt = nullptr;
  void *unit = nullptr;
  int inCh = 2, outCh = 2;
};

struct Engine {
  Slot slots[kMaxUnits];
  /* Which slot the audio thread renders. -1 = pass through. */
  std::atomic<int> active{-1};

  std::vector<float> main, sidechain;
  size_t frameCount = 0;

  std::atomic<size_t> playhead{0};
  std::atomic<bool> looping{true};
  std::atomic<bool> running{false};
  std::atomic<bool> bypass{false};

  std::atomic<float> inL{0}, inR{0}, outL{0}, outR{0};
  /* Fraction of output samples at or over full scale in the last block -
     distinct from the drive stage clipping, which the unit reports itself. */
  std::atomic<float> outOver{0};

  AudioQueueRef queue = nullptr;
  AudioQueueBufferRef buffers[kQueueBuffers] = {};

  float inBlock[kFramesPerBuffer * kMaxChannels];
  float outBlock[kFramesPerBuffer * 2];
};

Engine g;

void renderInto(float *dst, uint32_t frames) {
  const bool haveAudio = g.frameCount > 0;
  /* Latch the slot once per call: a close on the UI thread must not swap it
     out mid-buffer. */
  const int slot = g.active.load(std::memory_order_acquire);
  const Slot *s = (slot >= 0 && slot < kMaxUnits) ? &g.slots[slot] : nullptr;
  if (s && !s->unit) s = nullptr;
  const int inCh = s ? s->inCh : 2;
  float ipL = 0, ipR = 0, opL = 0, opR = 0;
  long overs = 0, outSamples = 0;

  uint32_t done = 0;
  while (done < frames) {
    const uint32_t n = (frames - done) < kFramesPerBuffer ? (frames - done) : kFramesPerBuffer;
    const size_t pos = g.playhead.load(std::memory_order_relaxed);

    for (uint32_t i = 0; i < n; ++i) {
      float mL = 0, mR = 0, sL = 0, sR = 0;
      if (haveAudio) {
        const size_t idx = (pos + i) % g.frameCount;
        mL = g.main[idx * 2]; mR = g.main[idx * 2 + 1];
        if (!g.sidechain.empty()) {
          const size_t s = idx % (g.sidechain.size() / 2);
          sL = g.sidechain[s * 2]; sR = g.sidechain[s * 2 + 1];
        } else { sL = mL; sR = mR; }
      }
      float *slot = &g.inBlock[i * inCh];
      slot[0] = mL;
      if (inCh > 1) slot[1] = mR;
      if (inCh > 3) { slot[2] = sL; slot[3] = sR; }
      const float pl = fabsf(mL), pr = fabsf(mR);
      if (pl > ipL) ipL = pl;
      if (pr > ipR) ipR = pr;
    }

    if (g.bypass.load(std::memory_order_relaxed) || !s) {
      for (uint32_t i = 0; i < n; ++i) {
        g.outBlock[i * 2] = g.inBlock[i * inCh];
        g.outBlock[i * 2 + 1] = inCh > 1 ? g.inBlock[i * inCh + 1] : g.inBlock[i * inCh];
      }
    } else {
      s->vt->render(s->unit, g.inBlock, g.outBlock, n);
    }

    for (uint32_t i = 0; i < n; ++i) {
      const float l = g.outBlock[i * 2], r = g.outBlock[i * 2 + 1];
      dst[(done + i) * 2] = l;
      dst[(done + i) * 2 + 1] = r;
      const float pl = fabsf(l), pr = fabsf(r);
      if (pl > opL) opL = pl;
      if (pr > opR) opR = pr;
      if (pl >= 1.0f) ++overs;
      if (pr >= 1.0f) ++overs;
      outSamples += 2;
    }

    if (haveAudio) {
      size_t next = pos + n;
      if (next >= g.frameCount)
        next = g.looping.load(std::memory_order_relaxed) ? next % g.frameCount : g.frameCount - 1;
      g.playhead.store(next, std::memory_order_relaxed);
    }
    done += n;
  }

  g.inL.store(ipL, std::memory_order_relaxed);
  g.inR.store(ipR, std::memory_order_relaxed);
  g.outL.store(opL, std::memory_order_relaxed);
  g.outR.store(opR, std::memory_order_relaxed);
  g.outOver.store(outSamples ? (float)overs / (float)outSamples : 0.f,
                  std::memory_order_relaxed);
}

void queueCallback(void *, AudioQueueRef q, AudioQueueBufferRef buf) {
  const uint32_t frames = buf->mAudioDataBytesCapacity / (2 * sizeof(float));
  renderInto(reinterpret_cast<float *>(buf->mAudioData), frames);
  buf->mAudioDataByteSize = frames * 2 * sizeof(float);
  AudioQueueEnqueueBuffer(q, buf, 0, nullptr);
}

bool readWav(const char *path, std::vector<float> &out, size_t &frames) {
  FILE *f = fopen(path, "rb");
  if (!f) return false;
  auto rd32 = [&](uint32_t &v) { return fread(&v, 4, 1, f) == 1; };
  auto rd16 = [&](uint16_t &v) { return fread(&v, 2, 1, f) == 1; };
  char riff[4], wave[4]; uint32_t sz;
  if (fread(riff,1,4,f)!=4 || !rd32(sz) || fread(wave,1,4,f)!=4 ||
      memcmp(riff,"RIFF",4) || memcmp(wave,"WAVE",4)) { fclose(f); return false; }
  uint16_t fmt=0, channels=0, bits=0; bool haveFmt=false;
  std::vector<uint8_t> data;
  while (!feof(f)) {
    char id[4]; uint32_t csz;
    if (fread(id,1,4,f)!=4 || !rd32(csz)) break;
    if (!memcmp(id,"fmt ",4)) {
      uint16_t ba; uint32_t rate, br;
      rd16(fmt); rd16(channels); rd32(rate); rd32(br); rd16(ba); rd16(bits);
      if (csz > 16) fseek(f, csz-16, SEEK_CUR);
      haveFmt = true;
    } else if (!memcmp(id,"data",4)) {
      data.resize(csz);
      if (fread(data.data(),1,csz,f)!=csz) { fclose(f); return false; }
    } else fseek(f, (csz+1)&~1u, SEEK_CUR);
  }
  fclose(f);
  if (!haveFmt || data.empty() || !channels) return false;
  const size_t bps = bits/8, total = data.size()/bps;
  frames = total/channels;
  out.assign(frames*2, 0.f);
  for (size_t i=0;i<frames;++i)
    for (int ch=0; ch<2; ++ch) {
      const uint8_t *p = data.data() + (i*channels + (channels==1?0:(size_t)ch))*bps;
      float v = 0;
      if (fmt==3 && bits==32) memcpy(&v,p,4);
      else if (bits==16) { int16_t s; memcpy(&s,p,2); v = s/32768.f; }
      else if (bits==24) { int32_t s = (p[0]<<8)|(p[1]<<16)|(p[2]<<24); v = s/2147483648.f; }
      else if (bits==32) { int32_t s; memcpy(&s,p,4); v = s/2147483648.f; }
      out[i*2+ch] = v;
    }
  return true;
}

bool writeWav(const char *path, const std::vector<float> &d, size_t frames) {
  FILE *f = fopen(path, "wb"); if (!f) return false;
  const uint32_t db=(uint32_t)(frames*8), riff=36+db, rate=kSampleRate, br=rate*8, fs=16;
  const uint16_t fmt=3, ch=2, al=8, bits=32;
  fwrite("RIFF",1,4,f); fwrite(&riff,4,1,f); fwrite("WAVE",1,4,f);
  fwrite("fmt ",1,4,f); fwrite(&fs,4,1,f); fwrite(&fmt,2,1,f); fwrite(&ch,2,1,f);
  fwrite(&rate,4,1,f); fwrite(&br,4,1,f); fwrite(&al,2,1,f); fwrite(&bits,2,1,f);
  fwrite("data",1,4,f); fwrite(&db,4,1,f); fwrite(d.data(),1,db,f);
  fclose(f); return true;
}

const cdh_vtable *vtableFor(int platform) {
  switch (platform) {
    case CDH_PLATFORM_DRUMLOGUE: return &cdh_vtable_drumlogue;
    case CDH_PLATFORM_NTS3: return &cdh_vtable_nts3;
    default: return nullptr;
  }
}

}  // namespace

extern "C" {

int cdh_platform_count() { return CDH_NUM_PLATFORMS; }
const char *cdh_platform_name(int i) {
  const cdh_vtable *v = vtableFor(i);
  return v ? v->platform_name : "";
}

/* Returns the slot index the unit was loaded into, or a negative error. */
int cdh_open(int platform, const char *dylibPath, char *err, int errlen) {
  const cdh_vtable *vt = vtableFor(platform);
  if (!vt) { snprintf(err, errlen, "unknown platform %d", platform); return -1; }

  int slot = -1;
  for (int i = 0; i < kMaxUnits; ++i)
    if (!g.slots[i].unit) { slot = i; break; }
  if (slot < 0) { snprintf(err, errlen, "no free slot (max %d units)", kMaxUnits); return -3; }

  err[0] = '\0';
  void *u = vt->open(dylibPath, kSampleRate, kFramesPerBuffer, err, errlen);
  if (!u) return -2;

  g.slots[slot].vt = vt;
  g.slots[slot].inCh = vt->in_channels(u);
  g.slots[slot].outCh = vt->out_channels(u);
  g.slots[slot].unit = u;                       /* publish last */

  if (g.active.load() < 0) g.active.store(slot, std::memory_order_release);
  return slot;
}

static bool validSlot(int i) {
  return i >= 0 && i < kMaxUnits && g.slots[i].unit != nullptr;
}

void cdh_close_slot(int slot) {
  if (!validSlot(slot)) return;
  /* Take it off the audio thread before tearing it down. */
  if (g.active.load() == slot) {
    int next = -1;
    for (int i = 0; i < kMaxUnits; ++i)
      if (i != slot && g.slots[i].unit) { next = i; break; }
    g.active.store(next, std::memory_order_release);
  }
  Slot &s = g.slots[slot];
  void *u = s.unit;
  s.unit = nullptr;
  s.vt->close(u);
  s.vt = nullptr;
}

void cdh_close_all() { for (int i = 0; i < kMaxUnits; ++i) cdh_close_slot(i); }

int cdh_max_units() { return kMaxUnits; }
void cdh_set_active(int slot) {
  if (slot < 0 || validSlot(slot)) g.active.store(slot, std::memory_order_release);
}
int cdh_get_active() { return g.active.load(); }

const char *cdh_unit_name(int slot) { return validSlot(slot) ? g.slots[slot].vt->name(g.slots[slot].unit) : ""; }
const char *cdh_unit_platform(int slot) { return validSlot(slot) ? g.slots[slot].vt->platform_name : ""; }
int cdh_param_count(int slot) { return validSlot(slot) ? g.slots[slot].vt->param_count(g.slots[slot].unit) : 0; }
int cdh_param_get(int slot, int i, cdh_param *o) {
  return validSlot(slot) ? g.slots[slot].vt->param_get(g.slots[slot].unit, i, o) : -1;
}
void cdh_set_param(int slot, int id, int32_t v) {
  if (validSlot(slot)) g.slots[slot].vt->set_param(g.slots[slot].unit, id, v);
}
int32_t cdh_get_param(int slot, int id) {
  return validSlot(slot) ? g.slots[slot].vt->get_param(g.slots[slot].unit, id) : 0;
}
int cdh_in_channels(int slot) { return validSlot(slot) ? g.slots[slot].inCh : 0; }

int cdh_get_bitmap(int slot, int id, uint8_t *out32) {
  if (!validSlot(slot) || !g.slots[slot].vt->get_bitmap || !out32) return -1;
  const uint8_t *b = g.slots[slot].vt->get_bitmap(g.slots[slot].unit, id);
  if (!b) return -1;
  memcpy(out32, b, 32);
  return 0;
}

int cdh_has_touch(int slot) { return (validSlot(slot) && g.slots[slot].vt->touch) ? 1 : 0; }
void cdh_touch(int slot, int id, float x, float y, int state) {
  if (validSlot(slot) && g.slots[slot].vt->touch)
    g.slots[slot].vt->touch(g.slots[slot].unit, id, x, y, state);
}
void cdh_set_tempo(int slot, float bpm) {
  if (validSlot(slot) && g.slots[slot].vt->set_tempo)
    g.slots[slot].vt->set_tempo(g.slots[slot].unit, bpm);
}

int cdh_load_wav(const char *p) {
  std::vector<float> b; size_t n = 0;
  if (!readWav(p, b, n) || !n) return -1;
  g.main.swap(b); g.frameCount = n; g.playhead.store(0);
  return 0;
}
int cdh_load_sidechain_wav(const char *p) {
  std::vector<float> b; size_t n = 0;
  if (!readWav(p, b, n) || !n) return -1;
  g.sidechain.swap(b); return 0;
}
void cdh_clear_sidechain() { g.sidechain.clear(); }

int cdh_start_audio() {
  if (g.running.load()) return 0;
  AudioStreamBasicDescription f; memset(&f, 0, sizeof(f));
  f.mSampleRate = kSampleRate; f.mFormatID = kAudioFormatLinearPCM;
  f.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
  f.mChannelsPerFrame = 2; f.mBitsPerChannel = 32;
  f.mBytesPerFrame = 8; f.mFramesPerPacket = 1; f.mBytesPerPacket = 8;
  if (AudioQueueNewOutput(&f, queueCallback, nullptr, nullptr, nullptr, 0, &g.queue) != noErr) return -2;
  for (int i = 0; i < kQueueBuffers; ++i) {
    if (AudioQueueAllocateBuffer(g.queue, kQueueFrames * 8, &g.buffers[i]) != noErr) return -3;
    queueCallback(nullptr, g.queue, g.buffers[i]);
  }
  if (AudioQueueStart(g.queue, nullptr) != noErr) return -4;
  g.running.store(true);
  return 0;
}
void cdh_stop_audio() {
  if (!g.running.load()) return;
  AudioQueueStop(g.queue, true); AudioQueueDispose(g.queue, true);
  g.queue = nullptr; g.running.store(false);
}
int cdh_is_running() { return g.running.load() ? 1 : 0; }
void cdh_set_looping(int on) { g.looping.store(on != 0); }
void cdh_set_bypass(int on) { g.bypass.store(on != 0); }
double cdh_duration() { return g.frameCount / (double)kSampleRate; }
double cdh_position() { return g.playhead.load() / (double)kSampleRate; }
void cdh_seek(double s) {
  if (!g.frameCount) return;
  long f = (long)(s * kSampleRate);
  if (f < 0) f = 0;
  if ((size_t)f >= g.frameCount) f = g.frameCount - 1;
  g.playhead.store((size_t)f);
}
/* Fraction of output samples at or over 0 dBFS in the last block. */
float cdh_get_output_over() { return g.outOver.load(std::memory_order_relaxed); }

int cdh_diag_count(int slot) {
  return validSlot(slot) && g.slots[slot].vt->diag_count
             ? g.slots[slot].vt->diag_count(g.slots[slot].unit) : 0;
}
const char *cdh_diag_get(int slot, int idx, float *value) {
  if (value) *value = 0.f;
  return validSlot(slot) && g.slots[slot].vt->diag_get
             ? g.slots[slot].vt->diag_get(g.slots[slot].unit, idx, value) : 0;
}

void cdh_get_meters(float *a, float *b, float *c, float *d) {
  if (a) *a = g.inL.load(); if (b) *b = g.inR.load();
  if (c) *c = g.outL.load(); if (d) *d = g.outR.load();
}

int cdh_render_offline(const char *outPath) {
  const int slot = g.active.load();
  if (!validSlot(slot) || !g.frameCount) return -1;
  const bool loop = g.looping.load(); const size_t pos = g.playhead.load();
  g.looping.store(false); g.playhead.store(0);
  g.slots[slot].vt->reset(g.slots[slot].unit);
  std::vector<float> out(g.frameCount * 2, 0.f);
  size_t done = 0;
  while (done < g.frameCount) {
    const uint32_t n = (uint32_t)((g.frameCount - done) < kFramesPerBuffer ? (g.frameCount - done)
                                                                          : kFramesPerBuffer);
    renderInto(out.data() + done * 2, n);
    done += n;
  }
  const bool ok = writeWav(outPath, out, g.frameCount);
  g.looping.store(loop); g.playhead.store(pos);
  g.slots[slot].vt->reset(g.slots[slot].unit);
  return ok ? 0 : -2;
}

}  // extern "C"

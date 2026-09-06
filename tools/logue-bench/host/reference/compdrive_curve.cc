// NOTE: targets the older single-unit harness ABI (cd_*), not the
// slot-addressed cdhost ABI. Kept because it is what verified the
// corrected compressor curve against the real unit; needs its calls
// updated to cdh_* before it will build again. Not part of any build.
// Verifies the compressor curve through the REAL unit code, not a replica.
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <vector>
extern "C" {
  int cd_init(); void cd_teardown();
  void cd_set_param(int,int32_t); int32_t cd_get_param(int);
  int cd_get_bitmap(int, uint8_t*);
  int cd_load_wav(const char*); int cd_render_offline(const char*);
}
static float db(float l){ return 20.0f*log10f(l<1e-9f?1e-9f:l); }
static void writeWav(const char* p, const std::vector<float>& x){
  FILE* f=fopen(p,"wb");
  uint32_t n=(uint32_t)(x.size()*4), riff=36+n, rate=48000, br=rate*8, fs=16;
  uint16_t fmt=3, ch=2, al=8, bits=32;
  fwrite("RIFF",1,4,f); fwrite(&riff,4,1,f); fwrite("WAVE",1,4,f);
  fwrite("fmt ",1,4,f); fwrite(&fs,4,1,f); fwrite(&fmt,2,1,f); fwrite(&ch,2,1,f);
  fwrite(&rate,4,1,f); fwrite(&br,4,1,f); fwrite(&al,2,1,f); fwrite(&bits,2,1,f);
  fwrite("data",1,4,f); fwrite(&n,4,1,f); fwrite(x.data(),1,n,f); fclose(f);
}
// RMS of the settled tail (skip the first 0.5 s while the envelope converges)
static float tailRms(const char* p){
  FILE* f=fopen(p,"rb"); if(!f) return 0;
  fseek(f, 44 + 24000*2*4, SEEK_SET);
  double acc=0; long n=0; float v;
  while(fread(&v,4,1,f)==1){ acc += (double)v*v; n++; }
  fclose(f);
  return n ? (float)sqrt(acc/n) : 0.f;
}
int main(){
  if (cd_init()){ printf("init failed\n"); return 1; }

  std::vector<float> sig(48000*2);
  for (int i=0;i<48000;i++){
    float s = 0.5f*sinf(2.f*(float)M_PI*200.f*i/48000.f);   // -6 dBFS peak
    sig[i*2]=s; sig[i*2+1]=s;
  }
  writeWav("/tmp/cd_in.wav", sig);
  cd_load_wav("/tmp/cd_in.wav");
  writeWav("/tmp/cd_ref.wav", sig);
  const float inRms = tailRms("/tmp/cd_ref.wav");
  printf("input: -6.02 dBFS peak, %.2f dBFS RMS\n", db(inRms));

  cd_set_param(8, 100);   // Dry/Wet fully wet, to isolate the compressor

  printf("\n  THold    Ratio  Knee    out RMS    gain     ideal    err\n");
  struct { int thold, ratio, knee; } cases[] = {
    {   0,  1, 0}, { -40,  2, 0}, { -40,  4, 0},
    { -40, 10, 0}, { -40, 20, 0}, { -40,  4,12}, { -60,  4, 0},
  };
  for (auto&c : cases){
    cd_set_param(2,c.thold); cd_set_param(3,c.ratio); cd_set_param(5,c.knee);
    cd_render_offline("/tmp/cd_out.wav");
    const float outRms = tailRms("/tmp/cd_out.wav");
    const float gain = db(outRms) - db(inRms);
    // ideal steady-state: envelope tracks the peak (0.5), gain from the curve
    const float T = c.thold*0.5f, R = (float)c.ratio;
    const float over = db(0.5f) - T;
    const float ideal = over > 0 ? (1.f/R - 1.f)*over : 0.f;
    printf("  %4d/%-5.1f %3d:1  %4.1f   %7.2f  %+7.2f  %+7.2f  %+5.2f\n",
           c.thold, T, c.ratio, c.knee*0.5f, db(outRms), gain, ideal,
           c.knee ? 0.f : gain-ideal);
  }
  cd_teardown();
  return 0;
}

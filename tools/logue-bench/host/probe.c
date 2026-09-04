/* Loads a unit dylib through the host and dumps what the bench would show. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "cdhost.h"

extern int cdh_open(int, const char*, char*, int);
extern void cdh_close_slot(int);
extern const char* cdh_unit_name(int);
extern const char* cdh_unit_platform(int);
extern int cdh_param_count(int);
extern int cdh_param_get(int, int, cdh_param*);
extern int32_t cdh_get_param(int, int);
extern int cdh_in_channels(int);
extern int cdh_has_touch(int);
extern int cdh_get_bitmap(int, int, uint8_t*);
extern int cdh_get_active(void);
extern void cdh_set_active(int);
extern int cdh_platform_count(void);
extern const char* cdh_platform_name(int);

static const char* tn(uint8_t t){
  static const char* n[]={"none","percent","db","cents","semi","oct","hertz","khertz","bpm",
    "msec","sec","enum","strings","bitmaps","drywet","pan","spread","onoff","midi_note"};
  return t < sizeof(n)/sizeof(*n) ? n[t] : "?";
}
static int load(int plat, const char* path){
  char err[256] = {0};
  int slot = cdh_open(plat, path, err, sizeof err);
  printf("\n=== slot %d :: platform '%s' :: %s\n", slot, cdh_platform_name(plat),
         strrchr(path,'/')+1);
  if (slot < 0) { printf("  FAILED: %s\n", err); return 1; }
  printf("  unit name   : \"%s\" (%s)\n", cdh_unit_name(slot), cdh_unit_platform(slot));
  printf("  in channels : %d\n", cdh_in_channels(slot));
  printf("  touch       : %s\n", cdh_has_touch(slot)? "yes (Kaoss X/Y)" : "no");
  uint8_t b[32];
  printf("  bitmap param: %s\n", cdh_get_bitmap(slot,4,b)==0 ? "yes" : "none");
  printf("  params      : %d\n", cdh_param_count(slot));
  for (int i=0;i<cdh_param_count(slot);i++){
    cdh_param p;
    if (cdh_param_get(slot,i,&p)) continue;
    if (!p.name[0]) { printf("    %2d  (blank)\n", i); continue; }
    printf("    %2d  %-22s %-8s %6d..%-6d init %-5d frac %d  now %d\n",
           i, p.name, tn(p.type), p.min, p.max, p.init, p.frac, cdh_get_param(slot,i));
  }
  return 0;
}
int main(int argc, char** argv){
  printf("platforms registered: %d\n", cdh_platform_count());
  for (int i=0;i<cdh_platform_count();i++) printf("  %d = %s\n", i, cdh_platform_name(i));
  int rc = 0;
  rc |= load(CDH_PLATFORM_DRUMLOGUE, argv[1]);
  rc |= load(CDH_PLATFORM_NTS3,      argv[2]);
  printf("\nboth units held open simultaneously; active slot = %d\n", cdh_get_active());
  cdh_set_active(1);
  printf("switched active to %d (\"%s\")\n", cdh_get_active(), cdh_unit_name(cdh_get_active()));
  cdh_close_slot(0); cdh_close_slot(1);
  return rc;
}

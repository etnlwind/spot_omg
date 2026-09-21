"""Physical-coordinate per-foot lift, independent settings and exact zero mode."""
from pathlib import Path
import subprocess
from servo.host_build import build_executable

ROOT=Path(__file__).resolve().parents[3]

def test_cartesian_lift_all_deployed_non_native_profiles(tmp_path):
    source=tmp_path/'lift.c'
    source.write_text(r'''
#undef NDEBUG
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "foot_lift.h"
#include "locomotion_servo.h"
int main(void){
 unsigned cases=0,unreachable=0;
 for(int profile=0;profile<LOCOMOTION_PROFILE_COUNT;profile++){
  if(locomotion_is_native(profile))continue;
  for(int d=-1;d<=1;d+=2)for(int t=0;t<100;t++){
   GaitPolicyLegTarget base[4],out[4];float phase=t*.01f;
   assert(locomotion_targets(profile,phase,1,d,0,base));
   uint32_t zero[4]={0};memcpy(out,base,sizeof out);
   assert(foot_lift_profile(zero,profile,phase,1,d,0,out));assert(memcmp(base,out,sizeof out)==0);
   for(int chosen=0;chosen<4;chosen++){
    uint32_t mm[4]={0};mm[chosen]=10;memcpy(out,base,sizeof out);
    if(!foot_lift_profile(mm,profile,phase,1,d,0,out)){if(unreachable<12)fprintf(stderr,"unreachable %s d=%d phase=%f leg=%d\n",locomotion_names[profile],d,phase,chosen);if(unreachable==0)for(int k=0;k<4;k++)fprintf(stderr,"q %f %f %f\n",base[k].j1_deg,base[k].j2_deg,base[k].j3_deg);unreachable++;continue;}
    for(int leg=0;leg<4;leg++){
     if(leg!=chosen){assert(memcmp(base+leg,out+leg,sizeof *out)==0);continue;}
     float a[3]={base[leg].j1_deg,base[leg].j2_deg,base[leg].j3_deg},b[3]={out[leg].j1_deg,out[leg].j2_deg,out[leg].j3_deg},fa[3],fb[3];
     arc_foot(leg,a,fa,0);arc_foot(leg,b,fb,0);
     assert(fabsf(fa[0]-fb[0])<.000011f && fabsf(fa[1]-fb[1])<.000011f);
     assert(fb[2]-fa[2]>-.000011f && fb[2]-fa[2]<.010011f);
     if(base[leg].stance)assert(memcmp(base+leg,out+leg,sizeof *out)==0);
    }
    uint16_t ticks[12];assert(locomotion_servo_targets(out,ticks));cases++;
   }
  }
 }
 printf("%u valid cases, %u rejected unreachable cases\n",cases,unreachable);
 assert(unreachable==0);
}
''')
    exe=build_executable([source,ROOT/'firmware/stm32-learning/Src/robot_config.c'],ROOT/'firmware/stm32-learning/Inc',tmp_path/'lift')
    subprocess.run([str(exe)],check=True)


def test_lift_above_10_and_unreachable_target_is_atomic(tmp_path):
    source=tmp_path/'large_lift.c'
    source.write_text(r'''
#undef NDEBUG
#include <assert.h>
#include <string.h>
#include "foot_lift.h"
int main(void){
 GaitPolicyLegTarget base[4],out[4];
 for(int i=0;i<4;i++){base[i].j1_deg=0;base[i].j2_deg=45;base[i].j3_deg=90;base[i].stance=false;}
 float offsets[4]={0,0,0,0};
 for(unsigned mm=20;mm<=30;mm+=10){
  uint32_t lift[4]={mm,0,0,0};memcpy(out,base,sizeof out);
  assert(foot_lift_apply(lift,.75f,.5f,1,offsets,out));
  float a[3]={0,45,90},b[3]={out[0].j1_deg,out[0].j2_deg,out[0].j3_deg},fa[3],fb[3];
  arc_foot(0,a,fa,0);arc_foot(0,b,fb,0);
  assert(fabsf(fb[2]-fa[2]-mm/1000.f)<.000011f);
  assert(fabsf(fb[0]-fa[0])<.000011f && fabsf(fb[1]-fa[1])<.000011f);
 }
 uint32_t impossible[4]={2147483647U,0,0,0};memcpy(out,base,sizeof out);
 assert(!foot_lift_apply(impossible,.75f,.5f,1,offsets,out));
 assert(memcmp(base,out,sizeof out)==0);
}
''')
    exe=build_executable([source],ROOT/'firmware/stm32-learning/Inc',tmp_path/'large_lift')
    subprocess.run([str(exe)],check=True)

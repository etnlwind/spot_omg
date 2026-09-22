from pathlib import Path
import subprocess
from servo.host_build import build_executable
ROOT=Path(__file__).resolve().parents[3]

def test_width_xyz_independence_zero_and_unreachable(tmp_path):
    source=tmp_path/'width.c'
    source.write_text(r'''
#undef NDEBUG
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "foot_lift.h"
#include "locomotion_servo.h"
int main(void){
 unsigned cases=0;
 for(int profile=0;profile<LOCOMOTION_PROFILE_COUNT;profile++){
  if(locomotion_is_native(profile))continue;
  for(int d=-1;d<=1;d+=2)for(int t=0;t<40;t++){
   GaitPolicyLegTarget base[4],out[4];
   assert(locomotion_targets(profile,t*.025f,1,d,0,base));
   int32_t zero[4]={0};memcpy(out,base,sizeof out);
   assert(foot_width_apply(zero,1,out));assert(!memcmp(base,out,sizeof out));
   for(int chosen=0;chosen<4;chosen++)for(int sign=-1;sign<=1;sign+=2){
    int32_t mm[4]={0};mm[chosen]=sign*5;memcpy(out,base,sizeof out);
    if(!foot_width_apply(mm,1,out)){fprintf(stderr,"unreachable %s %d %d %d %d\n",locomotion_names[profile],d,t,chosen,sign);return 2;}
    for(int leg=0;leg<4;leg++){
     if(leg!=chosen){assert(!memcmp(base+leg,out+leg,sizeof *out));continue;}
     float a[3]={base[leg].j1_deg,base[leg].j2_deg,base[leg].j3_deg},b[3]={out[leg].j1_deg,out[leg].j2_deg,out[leg].j3_deg},fa[3],fb[3];
     /* Independent physical motor->CAD convention from measured front J1.
      * The old implementation fails this physical-frame test. */
     if(leg<2){a[0]=-a[0];b[0]=-b[0];}
     arc_foot(leg,a,fa,0);arc_foot(leg,b,fb,0);
     assert(fabsf(fa[0]-fb[0])<.000011f && fabsf(fa[2]-fb[2])<.000011f);
     assert(fabsf(fb[1]-fa[1]-(leg%2==0?1:-1)*sign*.005f)<.000011f);
    }
    uint16_t ticks[12],before[12];assert(locomotion_servo_targets(out,ticks));
    assert(locomotion_servo_targets(base,before));
    int delta=(int)ticks[chosen*3]-(int)before[chosen*3];
    /* Inward: FL ID1 decreases, FR ID4 increases; rear is opposite. */
    int outward_tick_sign=chosen==0||chosen==3?1:-1;
    assert(delta*sign*outward_tick_sign>0);cases++;
   }
   int32_t bad[4]={0,0,0,2147483647};memcpy(out,base,sizeof out);
   assert(!foot_width_apply(bad,1,out));assert(!memcmp(base,out,sizeof out));
  }
 }
 printf("%u width cases passed\n",cases);
}
''')
    exe=build_executable([source,ROOT/'firmware/stm32-learning/Src/robot_config.c'],ROOT/'firmware/stm32-learning/Inc',tmp_path/'width')
    subprocess.run([str(exe)],check=True)

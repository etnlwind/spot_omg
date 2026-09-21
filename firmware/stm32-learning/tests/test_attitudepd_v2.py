"""Shared C clearance contracts; not evidence of physical foot clearance."""
import subprocess
from servo.host_build import build_executable
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

def test_v2_front_clearance_and_rear_preservation(tmp_path):
    source = r'''#include <assert.h>
#include <math.h>
#include <string.h>
#include "locomotion.h"
int main(void) {
 int old=locomotion_profile_id("attitudepd"), v2=locomotion_profile_id("attitudepd_v2");
 assert(old==15 && v2==24 && LOCOMOTION_DEFAULT_PROFILE==locomotion_profile_id("attitudepd_v6"));
 for(int c=-10;c<=10;c++)for(int t=0;t<=200;t++) {
  float linear=c/10.f,phase=t/200.f;
  GaitPolicyLegTarget a[4],b[4];
  assert(locomotion_targets(old,phase,1,linear,0,a));
  assert(locomotion_targets(v2,phase,1,linear,0,b));
  for(int i=0;i<4;i++) {
   assert(a[i].stance==b[i].stance);
   if(i>=2 || a[i].stance || linear<=0) {
    assert(a[i].j1_deg==b[i].j1_deg && a[i].j2_deg==b[i].j2_deg && a[i].j3_deg==b[i].j3_deg);
   } else {
    float qa[3]={a[i].j1_deg,a[i].j2_deg,a[i].j3_deg},qb[3]={b[i].j1_deg,b[i].j2_deg,b[i].j3_deg},fa[3],fb[3],p[7];
    arc_foot(i,qa,fa,0);arc_foot(i,qb,fb,0);locomotion_params(v2,linear,p);
    float dz=.004f*gait_policy_smootherstep(gait_policy_clampf(linear/.5f,0,1))*arc_swing_shape_envelope(phase+(i==1?.5f:0),p[1]);
    assert(fabsf(fb[0]-fa[0])<1.e-5f && fabsf(fb[1]-fa[1])<1.e-5f);
    assert(fabsf(fb[2]-fa[2]-dz)<1.e-5f);
   }
  }
 }
 for(int t=0;t<=100;t++) {
  GaitPolicyLegTarget q[4];float s=t/100.f;
  assert(locomotion_targets(v2,.8f,s,s,.2f*s,q));
  if(t==0)for(int i=0;i<4;i++) {
   assert(q[i].j1_deg==sn_standing[i][0]);assert(q[i].j2_deg==sn_standing[i][1]);assert(q[i].j3_deg==sn_standing[i][2]);
  }
 }
 return 0;
}
'''
    c=tmp_path/'test.c';c.write_text(source)
    exe=build_executable([c],ROOT/'firmware/stm32-learning/Inc',tmp_path/'test',
                         extra=('-ffp-contract=off',))
    subprocess.run([str(exe)],check=True)

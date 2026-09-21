"""V5 retains V4 geometry and increases propulsion by advancing gait faster."""
import json
import subprocess
from pathlib import Path
from servo.host_build import build_executable
ROOT=Path(__file__).resolve().parents[3]


def test_v5_deployed_kernel(tmp_path):
    source=tmp_path/'v5.c'
    source.write_text(r'''
#include <assert.h>
#include <stdio.h>
#include "drive_control.h"
#include "foot_lift.h"
#include "locomotion_servo.h"
static void same(GaitPolicyLegTarget a[4],GaitPolicyLegTarget b[4]) {
 for(int i=0;i<4;i++){
  assert(a[i].j1_deg==b[i].j1_deg && a[i].j2_deg==b[i].j2_deg && a[i].j3_deg==b[i].j3_deg);
  assert(a[i].stance==b[i].stance);
 }
}
int main(void) {
 int v4=locomotion_profile_id("attitudepd_v4"),v5=locomotion_profile_id("attitudepd_v5");
 assert(v4==26 && v5==27 && LOCOMOTION_DEFAULT_PROFILE==locomotion_profile_id("attitudepd_v6"));
 assert(locomotion_is_attitude_pd(v5) && locomotion_has_gait_stand(v5));
 GaitPolicyLegTarget a[4],b[4],ready[4];
 assert(locomotion_stand_targets(v4,a) && locomotion_stand_targets(v5,ready));same(a,ready);
 unsigned frames=0;
 const uint32_t lifts[3][4]={{0,0,0,0},{30,30,0,0},{1,7,13,3}};
 for(int l=-10;l<=10;l++)for(int y=-5;y<=5;y++)for(int n=0;n<100;n++){
  float linear=l*.1f,yaw=y*.1f,phase=n*.01f;
  float t4=locomotion_period(v4,linear,yaw),t5=locomotion_period(v5,linear,yaw);
  if(linear<=0)assert(t4==t5);
  else {
   assert(t5<=t4);
   if(linear>=.588f)assert(fabsf(t4/t5-locomotion_forward_cadence_gain[v5])<.000002f);
  }
  for(int s=0;s<=2;s++){
   float scale=s*.5f;
   assert(locomotion_targets(v4,phase,scale,linear,yaw,a));
   assert(locomotion_targets(v5,phase,scale,linear,yaw,b));same(a,b);
   float activity=scale*fminf(1,(fabsf(linear)+fabsf(yaw))/.15f);
   assert(foot_lift_profile(lifts[s],v4,phase,activity,linear,yaw,a));
   assert(foot_lift_profile(lifts[s],v5,phase,activity,linear,yaw,b));same(a,b);
   uint16_t ticks[12];assert(locomotion_servo_targets(b,ticks));frames++;
  }
 }
 /* Same phase increment contract and final B with changed cadence, through
    startup, forward/reverse/pivot changes, and requested stop. */
 DriveControl state={0};const float linear[]={.588f,1,-1,0,0},yaw[]={0,0,0,.5f,0};
 for(int s=0;s<5;s++)for(int i=0;i<150;i++){
  float before=state.phase;
  assert(drive_control_step_timed(&state,v5,linear[s],yaw[s],0,false,false,s==4,.02f,1,b));
  assert(fabsf(state.phase-fmodf(before+.02f/locomotion_period(v5,state.linear,state.yaw),1))<.000001f);
  if(s==4 && i>40)for(int j=0;j<4;j++){
   assert(fabsf(b[j].j1_deg-ready[j].j1_deg)<1.e-5f && fabsf(b[j].j2_deg-ready[j].j2_deg)<1.e-5f && fabsf(b[j].j3_deg-ready[j].j3_deg)<1.e-5f);
  }
 }
 DriveControl r4={0},r5={0};
 for(int n=0;n<400;n++){
  float l=n<200?-1:0,y=n<200?0:.5f;
  assert(drive_control_step_timed(&r4,v4,l,y,0,false,false,false,.02f,1,a));
  assert(drive_control_step_timed(&r5,v5,l,y,0,false,false,false,.02f,1,b));
  same(a,b);assert(r4.phase==r5.phase);
 }
 printf("V5 %u V4-identical geometry/lift cases, 750 drive frames, forward cadence gain verified; reverse unchanged\n",frames);
}
''')
    binary=build_executable([source,ROOT/'firmware/stm32-learning/Src/robot_config.c'],ROOT/'firmware/stm32-learning/Inc',tmp_path/'v5',extra=('-ffp-contract=off',))
    subprocess.run([str(binary)],check=True)


def test_v5_manifest_only_changes_forward_cadence_from_v4():
    m=json.loads((ROOT/'config/locomotion_profiles.json').read_text())
    a,b=m['profiles']['attitudepd_v4'],m['profiles']['attitudepd_v5']
    assert m['default']=='attitudepd_v6'
    assert list(m['profiles']).index('attitudepd_v5')==26
    for key in ('params','turn_reverse_params'):
        assert a[key]==b[key]
    assert a['half_stick_linear']==b['half_stick_linear']
    assert b['forward_cadence_gain']==1.3

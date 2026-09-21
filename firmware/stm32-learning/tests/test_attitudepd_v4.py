"""V4 direction-independent template selection through the deployed C kernel."""
import json
import subprocess
from pathlib import Path

from servo.host_build import build_executable

ROOT = Path(__file__).resolve().parents[3]


def test_half_stick_boundary_matches_app_mapping():
    from apps.windows.spot_controller.protocol import drive_vector
    profile = json.loads((ROOT/'config/locomotion_profiles.json').read_text())['profiles']['attitudepd_v4']
    assert drive_vector(0, .5) == (588, 0)
    assert drive_vector(0, -.5) == (-588, 0)
    assert profile['half_stick_linear'] == 588/1000
    assert drive_vector(0, 1) == (1000, 0)
    assert drive_vector(0, -1) == (-1000, 0)


def test_shared_gait_selection_geometry_and_transitions(tmp_path):
    source = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "drive_control.h"
#include "locomotion_servo.h"
static void same(const GaitPolicyLegTarget a[4],const GaitPolicyLegTarget b[4],float tolerance) {
 for(int i=0;i<4;i++) {
  assert(fabsf(a[i].j1_deg-b[i].j1_deg)<=tolerance);
  assert(fabsf(a[i].j2_deg-b[i].j2_deg)<=tolerance);
  assert(fabsf(a[i].j3_deg-b[i].j3_deg)<=tolerance);
 }
}
static void planar(const GaitPolicyLegTarget *q,float *x,float *z) {
 float a=q->j2_deg*GAIT_POLICY_PI/180,b=q->j3_deg*GAIT_POLICY_PI/180;
 *x=-.141f*sinf(a)-.150f*sinf(a-b);
 *z=.141f*cosf(a)+.150f*cosf(a-b);
}
int main(void) {
 int old=locomotion_profile_id("attitudepd_v3"),v4=locomotion_profile_id("attitudepd_v4");
 assert(old==25 && v4==26 && LOCOMOTION_DEFAULT_PROFILE==locomotion_profile_id("attitudepd_v6"));
 assert(locomotion_is_attitude_pd(v4) && locomotion_has_gait_stand(v4));
 assert(locomotion_linear(v4,-1)==-1);
 GaitPolicyLegTarget a[4],b[4],ready[4],previous[4];
 assert(locomotion_stand_targets(old,a) && locomotion_stand_targets(v4,ready));same(a,ready,0);
 float half=locomotion_half_stick_linear[v4];
 assert(half==.588f && locomotion_forward_template_weight(v4,half)==1);
 assert(locomotion_forward_template_weight(v4,-half)==1);
 assert(locomotion_forward_template_weight(v4,1)==0 && locomotion_forward_template_weight(v4,-1)==0);
 assert(fabsf(locomotion_period(v4,half,0)-1.35f*(1.35f-.35f*half))<1.e-6f);
 assert(fabsf(locomotion_period(v4,1,0)-1.05f)<1.e-6f);
 for(int n=0;n<=1000;n++) {
  float l=n*.001f,p[7],r[7];locomotion_params(v4,l,p);locomotion_params(v4,-l,r);
  for(int j=0;j<7;j++)assert(p[j]==r[j]);
  assert(locomotion_period(v4,l,0)==locomotion_period(v4,-l,0));
  float w=locomotion_forward_template_weight(v4,l);assert(w>=0 && w<=1);
  if(l>half)assert(w<=locomotion_forward_template_weight(v4,l-.001f));
 }
 /* Endpoints really retain old templates, including swing shape and +4mm
    front lift. Pure turns and zero-amplitude B retain the prior behavior. */
 for(int t=0;t<=200;t++)for(int s=0;s<=2;s++) {
  float phase=t/200.f,scale=s/2.f;
  assert(locomotion_targets(old,phase,scale,half,0,a));
  assert(locomotion_targets(v4,phase,scale,half,0,b));same(a,b,0);
  assert(locomotion_targets(old,phase,scale,-1,0,a));
  assert(locomotion_targets(v4,phase,scale,-1,0,b));same(a,b,0);
  for(int y=-5;y<=5;y++) {
   assert(locomotion_targets(old,phase,scale,0,y*.1f,a));
   assert(locomotion_targets(v4,phase,scale,0,y*.1f,b));same(a,b,0);
  }
 }
 /* Mirroring direction changes the fore-aft task, not period, lift, or
    diagonal timing. Rear targets expose the two-link task directly. */
 const float levels[]={.25f,.5f,.588f,.794f,1.f};
 for(int n=0;n<5;n++)for(int t=0;t<200;t++) {
  float l=levels[n],phase=t/200.f,p[7];locomotion_params(v4,l,p);
  assert(locomotion_targets(v4,phase,1,l,0,a));
  assert(locomotion_targets(v4,phase,1,-l,0,b));
  for(int i=0;i<4;i++)assert(a[i].stance==b[i].stance);
  assert(a[0].stance==a[3].stance && a[1].stance==a[2].stance);
  for(int i=2;i<4;i++) {
   float ax,az,bx,bz;planar(a+i,&ax,&az);planar(b+i,&bx,&bz);
   assert(fabsf(ax+bx-2*p[5])<1.e-6f && fabsf(az-bz)<1.e-6f);
  }
  for(int sign=-1;sign<=1;sign+=2) {
   float linear=sign*l,weight=locomotion_forward_template_weight(v4,linear);
   assert(center_pivot_targets_weighted(p,phase,1,linear,0,weight,a));
   assert(locomotion_targets(v4,phase,1,linear,0,b));
   for(int i=0;i<2;i++) {
    float qa[3]={a[i].j1_deg,a[i].j2_deg,a[i].j3_deg};
    float qb[3]={b[i].j1_deg,b[i].j2_deg,b[i].j3_deg},fa[3],fb[3];
    arc_foot(i,qa,fa,0);arc_foot(i,qb,fb,0);
    float dz=.004f*weight*arc_swing_shape_envelope(phase+(i==1?.5f:0),p[1]);
    assert(fabsf(fb[0]-fa[0])<1.e-5f && fabsf(fb[1]-fa[1])<1.e-5f);
    assert(fabsf(fb[2]-fa[2]-dz)<1.e-5f);
   }
  }
 }
 /* Smooth crossing of half/full stick, both signs and all phases. */
 const float boundaries[]={-1,-.588f,0,.588f,1};
 for(int j=0;j<5;j++)for(int t=0;t<100;t++) {
  float l=boundaries[j],lo=fmaxf(-1,l-1.e-4f),hi=fminf(1,l+1.e-4f);
  assert(locomotion_targets(v4,t*.01f,1,lo,0,a));
  assert(locomotion_targets(v4,t*.01f,1,hi,0,b));same(a,b,.02f);
 }
 /* Actual state machine: start, change mode, reverse, Stop. No phase reset,
    servo limits preserved, and zero-amplitude/idle targets remain B. */
 DriveControl state={0};uint16_t ticks[12];unsigned frames=0;
 const float request[]={half,1,half,-half,-1,0};
 for(int segment=0;segment<6;segment++)for(int t=0;t<150;t++) {
  float before=state.phase;
  assert(drive_control_step_timed(&state,v4,request[segment],0,0,false,false,segment==5,.02f,1,b));
  float expected=fmodf(before+.02f/locomotion_period(v4,state.linear,0),1.f);
  assert(fabsf(state.phase-expected)<1.e-6f);
  assert(locomotion_servo_targets(b,ticks));
  if(segment==5 && t>30)same(b,ready,1.e-5f);
  frames++;
 }
 for(int l=-10;l<=10;l++)for(int y=-5;y<=5;y++)for(int t=0;t<=100;t++) {
  assert(locomotion_targets(v4,t*.01f,0,l*.1f,y*.1f,b));same(b,ready,0);
  assert(locomotion_targets(v4,t*.01f,1,l*.1f,y*.1f,b));assert(locomotion_servo_targets(b,ticks));
 }
 printf("V4 endpoints, direction symmetry, clearance, continuity, %u drive frames and 23331 input/phase cases passed\n",frames);
}
'''
    unit = tmp_path/'v4.c'
    unit.write_text(source)
    binary = build_executable([unit, ROOT/'firmware/stm32-learning/Src/robot_config.c'],
        ROOT/'firmware/stm32-learning/Inc', tmp_path/'v4', extra=('-ffp-contract=off',))
    subprocess.run([str(binary)], check=True)

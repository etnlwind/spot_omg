"""V6 recovery contracts through production C; no physical robot I/O.

These verify commanded geometry, cadence and servo encoding. They do not
establish real clearance, contact timing or loaded actuator tracking.
"""
import subprocess
from pathlib import Path

from servo.host_build import build_executable

ROOT = Path(__file__).resolve().parents[3]


def test_v6_preserves_central_push_speed_with_longer_forward_recovery(tmp_path):
    source = tmp_path / 'v6.c'
    source.write_text(r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "drive_control.h"
#include "foot_lift.h"
#include "locomotion_servo.h"

static void same_leg(const GaitPolicyLegTarget *a,const GaitPolicyLegTarget *b) {
 assert(a->j1_deg==b->j1_deg && a->j2_deg==b->j2_deg && a->j3_deg==b->j3_deg);
 assert(a->stance==b->stance);
}
static void same_angles(const GaitPolicyLegTarget *a,const GaitPolicyLegTarget *b) {
 assert(a->j1_deg==b->j1_deg && a->j2_deg==b->j2_deg && a->j3_deg==b->j3_deg);
}
static float angle(const GaitPolicyLegTarget *target,int joint) {
 return joint==0?target->j1_deg:joint==1?target->j2_deg:target->j3_deg;
}
static void planar(const GaitPolicyLegTarget *target,float *x,float *z) {
 float a=target->j2_deg*GAIT_POLICY_PI/180,b=target->j3_deg*GAIT_POLICY_PI/180;
 *x=-.141f*sinf(a)-.150f*sinf(a-b);
 *z=.141f*cosf(a)+.150f*cosf(a-b);
}
static void sample(int profile,float phase,float scale,float linear,float yaw,
                   const uint32_t lift[4],GaitPolicyLegTarget out[4]) {
 assert(locomotion_targets(profile,phase,scale,linear,yaw,out));
 float activity=scale*fminf(1,(fabsf(linear)+fabsf(yaw))/.15f);
 assert(foot_lift_profile(lift,profile,phase,activity,linear,yaw,out));
 uint16_t ticks[12];assert(locomotion_servo_targets(out,ticks));
 for(int i=0;i<4;i++) {
  assert(isfinite(out[i].j1_deg) && out[i].j1_deg>=-30 && out[i].j1_deg<=30);
  assert(isfinite(out[i].j2_deg) && out[i].j2_deg>=-45 && out[i].j2_deg<=100);
  assert(isfinite(out[i].j3_deg) && out[i].j3_deg>=0 && out[i].j3_deg<=150);
 }
}

int main(void) {
 int v5=locomotion_profile_id("attitudepd_v5"),v6=locomotion_profile_id("attitudepd_v6");
 assert(v5==27 && v6==28); /* Older profile IDs and behavior stay fixed. */
 assert(locomotion_is_attitude_pd(v6) && locomotion_has_gait_stand(v6));
 assert(!locomotion_is_native(v6));
 const uint32_t lifts[2][4]={{0,0,0,0},{30,30,0,0}};
 GaitPolicyLegTarget old[4],new[4],ready[4],old_ready[4];
 assert(locomotion_stand_targets(v5,old_ready));
 assert(locomotion_stand_targets(v6,ready));
 for(int i=0;i<4;i++)same_leg(&ready[i],&old_ready[i]);
 unsigned samples=0,changed_swing=0;
 for(int l=-10;l<=10;l++)for(int y=-5;y<=5;y++)for(int n=0;n<100;n++) {
  float linear=l*.1f,yaw=y*.1f,phase=n*.01f;
  float period5=locomotion_period(v5,linear,yaw),period6=locomotion_period(v6,linear,yaw);
  if(linear<=0)assert(period6==period5);
  else {
   assert(period6>=period5 && period6<=period5*1.250001f);
   if(linear>=.15f)assert(fabsf(period6-period5*1.25f)<1.e-6f);
  }
  for(int s=0;s<=2;s++)for(int lift=0;lift<2;lift++) {
   float scale=s*.5f;
   sample(v5,phase,scale,linear,yaw,lifts[lift],old);
   sample(v6,phase,scale,linear,yaw,lifts[lift],new);
   assert(new[0].stance==new[3].stance && new[1].stance==new[2].stance);
   for(int leg=0;leg<4;leg++) {
    assert(new[leg].stance==old[leg].stance);
    if(scale==0)same_angles(&new[leg],&ready[leg]);
    if(linear<=0)same_leg(&old[leg],&new[leg]);
    if(linear>0 && !new[leg].stance && scale>0 &&
       (old[leg].j2_deg!=new[leg].j2_deg || old[leg].j3_deg!=new[leg].j3_deg))changed_swing++;
   }
   samples++;
  }
 }
 assert(changed_swing>0); /* A V5 alias must not pass as the new recovery. */

 /* Stance follows the same horizontal segment with a different clock.
    Test its monotonic path and endpoints independently of the shaping
    polynomial, then compare physical central velocity with V5. */
 const float forward_inputs[]={.3f,.588f,.8f,1.f};
 for(unsigned input=0;input<sizeof forward_inputs/sizeof *forward_inputs;input++) {
  float linear=forward_inputs[input],p[7];locomotion_params(v6,linear,p);
  float previous_x=INFINITY,first_x=0,last_x=0;
  for(int n=0;n<=100;n++) {
   sample(v6,p[1]*n/100.f,1,linear,0,lifts[0],new);
   float x,z;planar(&new[3],&x,&z);
   assert(fabsf(z-p[4])<1.e-6f);
   assert(x<=previous_x+1.e-6f);previous_x=x;
   if(n==0)first_x=x;
   if(n==100)last_x=x;
  }
  assert(fabsf(first_x-(p[5]+.5f*p[2]*linear))<1.e-6f);
  assert(fabsf(last_x-(p[5]-.5f*p[2]*linear))<1.e-6f);
  float speeds[2];
  for(int model=0;model<2;model++) {
   int profile=model?v6:v5;float xs[2];
   for(int side=0;side<2;side++) {
    sample(profile,p[1]*.5f+(side?1:-1)*.001f,1,linear,0,lifts[0],new);
    float z;planar(&new[3],&xs[side],&z);
   }
   speeds[model]=(xs[1]-xs[0])/(.002f*locomotion_period(profile,linear,0));
  }
  assert(speeds[0]<0 && fabsf(speeds[1]-speeds[0])<1.e-4f);
  float average5=(first_x-last_x)/(p[1]*locomotion_period(v5,linear,0));
  float average6=(first_x-last_x)/(p[1]*locomotion_period(v6,linear,0));
  assert(fabsf(average6/average5-.8f)<1.e-6f);
 }

 /* A two-second forward entry changes amplitude, not elapsed time.
    Reverse/pivot and all older profiles retain their one-second entry. */
 assert(locomotion_start_scale(v6,1,1)==.5f);
 assert(locomotion_start_scale(v6,2,1)==1);
 assert(locomotion_start_scale(v5,1,1)==1);
 for(int n=0;n<=250;n++)for(int l=-10;l<=10;l++) {
  float elapsed=n*.01f,linear=l*.1f;
  float a=locomotion_start_scale(v5,elapsed,linear),b=locomotion_start_scale(v6,elapsed,linear);
  assert(b>=0 && b<=1 && b<=a+1.e-6f);
  if(linear<=0)assert(a==b);
  if(linear>=.15f)assert(b==locomotion_start_scale(v5,elapsed*.5f,linear));
 }

 /* Both diagonal pairs cross liftoff and touchdown continuously. Use
    second-order endpoint estimates to check the new zero-speed contact
    endpoints without confusing finite sampling with a jump. Partial scale covers
    the real B-entry interpolation, and +30mm covers the user's adapter. */
 const float linear_inputs[]={.1f,.3f,.588f,.8f,1.f};
 const float offsets[4]={0,.5f,.5f,0};
 float worst_position=0,worst_velocity=0;
 for(unsigned input=0;input<sizeof linear_inputs/sizeof *linear_inputs;input++) {
  float linear=linear_inputs[input],p[7];locomotion_params(v6,linear,p);
  float period=locomotion_period(v6,linear,0);
  for(int s=0;s<=2;s++)for(int lift=0;lift<2;lift++)for(int leg=0;leg<4;leg++)for(int edge=0;edge<2;edge++) {
   const float h=.001f;
   float boundary=(edge==0?p[1]:1.f)-offsets[leg];
   GaitPolicyLegTarget q[5][4];
   for(int j=0;j<5;j++)sample(v6,boundary+(j-2)*h,s*.5f,linear,0,lifts[lift],q[j]);
   for(int joint=0;joint<3;joint++) {
    float m2=angle(&q[0][leg],joint),m1=angle(&q[1][leg],joint),zero=angle(&q[2][leg],joint);
    float p1=angle(&q[3][leg],joint),p2=angle(&q[4][leg],joint);
    float position_error=fabsf((2*m1-m2)-(2*p1-p2));
    float left=(3*zero-4*m1+m2)/(2*h*period),right=(-3*zero+4*p1-p2)/(2*h*period);
    float velocity_error=fabsf(left-right);
    worst_position=fmaxf(worst_position,position_error);worst_velocity=fmaxf(worst_velocity,velocity_error);
    if(position_error>.005f || velocity_error>1.5f) {
     fprintf(stderr,"V6 boundary linear=%g scale=%g lift=%d leg=%d edge=%d joint=%d position=%gdeg velocity=%gdeg/s\n",linear,s*.5f,lift,leg,edge,joint,position_error,velocity_error);
     return 1;
    }
   }
  }
 }

 /* Exercise the deployed state machine through forward, reverse, pivot and
    Stop. Phase follows the new period, elapsed remains real time, and
    the final posture remains B. */
 DriveControl state={0};
 const float requests[5][2]={{.588f,0},{1,0},{-1,0},{0,.5f},{0,0}};
 for(int segment=0;segment<5;segment++)for(int n=0;n<150;n++) {
  float before=state.phase,elapsed_before=state.elapsed;
  assert(drive_control_step_timed(&state,v6,requests[segment][0],requests[segment][1],0,false,false,segment==4,.02f,1,new));
  assert(fabsf(state.elapsed-elapsed_before-.02f)<1.e-6f);
  float expected=fmodf(before+.02f/locomotion_period(v6,state.linear,state.yaw),1.f);
  assert(fabsf(state.phase-expected)<1.e-6f);
  uint16_t ticks[12];assert(locomotion_servo_targets(new,ticks));
  if(segment==4 && n>40)for(int leg=0;leg<4;leg++)for(int joint=0;joint<3;joint++)
   assert(fabsf(angle(&new[leg],joint)-angle(&ready[leg],joint))<1.e-5f);
 }
 printf("V6 %u phase/input/scale/lift samples; V5 central push preserved, longer forward period/ramp, unchanged reverse/pivot; boundary errors <= %.6gdeg / %.6gdeg/s; 750 drive frames\n",samples,worst_position,worst_velocity);
}
''')
    binary = build_executable(
        [source, ROOT/'firmware/stm32-learning/Src/robot_config.c'],
        ROOT/'firmware/stm32-learning/Inc', tmp_path/'v6',
        extra=('-ffp-contract=off','-UNDEBUG'))
    subprocess.run([str(binary)], check=True)

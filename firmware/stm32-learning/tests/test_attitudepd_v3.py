"""Shared production C contracts for gait-ready Stand; no physical robot I/O."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
INC = ROOT/'firmware/stm32-learning/Inc'


def test_v3_stand_entry_stop_and_v2_steady_preservation(tmp_path):
    source = r'''
#include <assert.h>
#include <math.h>
#include "drive_control.h"
#include "attitude_pd.h"
#include "locomotion_servo.h"
static void same(GaitPolicyLegTarget a[4],GaitPolicyLegTarget b[4],float tolerance) {
 for(int i=0;i<4;i++) {
  assert(fabsf(a[i].j1_deg-b[i].j1_deg)<=tolerance);
  assert(fabsf(a[i].j2_deg-b[i].j2_deg)<=tolerance);
  assert(fabsf(a[i].j3_deg-b[i].j3_deg)<=tolerance);
 }
}
int main(void) {
 /* Regression: valid smoothing weights may never overshoot 1 by roundoff. */
 for(int t=0;t<=100000;t++) {
  float weights[4];assert(body_stabilizer_stance_weights(t/100000.f,.52f,1.23375f,.06f,weights));
  for(int i=0;i<4;i++)assert(weights[i]>=0 && weights[i]<=1);
 }
 int v2=locomotion_profile_id("attitudepd_v2"),v3=locomotion_profile_id("attitudepd_v3");
 assert(v2==24 && v3==25 && LOCOMOTION_DEFAULT_PROFILE==v3);
 assert(locomotion_is_attitude_pd(v3) && !locomotion_is_native(v3));
 GaitPolicyLegTarget a[4],b[4],stand[4],original[4];uint16_t ticks[12],old_ticks[12];
 assert(locomotion_stand_targets(v3,stand));
 assert(locomotion_targets(v2,0,1,0,0,a));same(a,stand,0);
 assert(locomotion_stand_targets(v2,original));
 assert(robot_stand_targets(old_ticks) && locomotion_servo_targets(original,ticks));
 for(int i=0;i<12;i++)assert(ticks[i]==old_ticks[i]);
 assert(fabsf(stand[0].j2_deg-original[0].j2_deg)>5);
 /* Every direction and phase shares B at zero amplitude; every full-scale
    target, including front clearance and turns, remains exactly V2. */
 for(int l=-10;l<=10;l++)for(int y=-5;y<=5;y++)for(int t=0;t<=100;t++) {
  float linear=l*.1f,yaw=y*.1f,phase=t*.01f;
  assert(locomotion_targets(v3,phase,0,linear,yaw,b));same(b,stand,0);
  assert(locomotion_targets(v2,phase,1,linear,yaw,a));
  assert(locomotion_targets(v3,phase,1,linear,yaw,b));same(a,b,0);
  for(int i=0;i<4;i++)assert(a[i].stance==b[i].stance);
 }
 /* Startup and commanded stop exercise the production slew/phase kernel.
    Once stopped, zero input at every elapsed time stays B rather than A/S. */
 const float request[4][2]={{1,0},{-1,0},{0,.5f},{0,-.5f}};
 for(int c=0;c<4;c++) {
  DriveControl state={0};
  for(int t=0;t<250;t++) {
   bool stopping=t>=150;
   assert(drive_control_step_timed(&state,v3,stopping?0:request[c][0],stopping?0:request[c][1],0,false,false,stopping,.02f,1,b));
   assert(locomotion_servo_targets(b,ticks));
   if(t==0)same(b,stand,.01f);
   if(t>=180)same(b,stand,1.e-5f);
  }
 }
 for(int t=0;t<=100;t++) {
  assert(locomotion_targets(v3,.75f,t*.01f,0,0,b));same(b,stand,1.e-5f);
 }
}
'''
    unit=tmp_path/'v3.c';unit.write_text(source);exe=tmp_path/'v3'
    subprocess.run(['clang','-O2','-ffp-contract=off','-I',str(INC),str(unit),
                    str(ROOT/'firmware/stm32-learning/Src/robot_config.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


def function(source, declaration):
    start=source.index(declaration);opening=source.index('{',start);level=1;end=opening+1
    while level:
        level += (source[end]=='{')-(source[end]=='}');end+=1
    return source[start:end]


def test_selected_stand_and_idle_use_same_servo_targets(tmp_path):
    source=(ROOT/'firmware/stm32-learning/Src/robot.c').read_text()
    selected=function(source,'bool robot_selected_stand_targets(')
    stand=function(source,'RobotResult robot_stand(')
    idle=function(source,'void robot_control_idle(')
    entry_start=source.index('    GaitPolicyLegTarget stand[4],neutral[4],nominal[4],from[4],command[4];')
    entry=source[entry_start:source.index('    unsigned step_half=',entry_start)]
    first_stop=function(source,'            if(stopping && fabsf(robot->drive_control.linear)')
    check_entry='static RobotResult check_entry(RobotController *robot) {\nconst bool native=false;RobotResult result;\n'+entry+r'''
 assert(stage==2);
 for(int i=0;i<4;i++)assert(command[i].j2_deg==stand[i].j2_deg && command[i].j3_deg==stand[i].j3_deg);
 stopping=true;
'''+first_stop+r'''
 assert(stage==3 && transition==0);
 for(int i=0;i<4;i++)assert(from[i].j2_deg==stand[i].j2_deg && nominal[i].j3_deg==stand[i].j3_deg);
 return ROBOT_OK;
}'''
    code=r'''
#include <assert.h>
#include <string.h>
#include "robot.h"
#include "sts3215.h"
#include "s_native_impl.h"
#include "s_native_servo.h"
#include "locomotion_servo.h"
static uint16_t commanded[12];static unsigned supervised, sent;
uint32_t HAL_GetTick(void){return 100;}
RobotResult robot_supervised_pose(RobotController *r,const uint16_t *q,bool is_stand) {
 (void)r;assert(is_stand);memcpy(commanded,q,sizeof commanded);supervised++;return ROBOT_OK;
}
static RobotResult shared_observe(RobotController *r){(void)r;return ROBOT_OK;}
static bool shared_correct(RobotController *r,GaitPolicyLegTarget *q,bool standing,bool moving) {
 (void)q;assert(locomotion_is_attitude_pd(r->locomotion_profile) && standing && !moving);return true;
}
bool robot_support_observe(RobotController *r,const uint16_t *q){(void)r;(void)q;return true;}
void robot_latch_locomotion_fault(RobotController *r,RobotResult result){(void)r;(void)result;assert(0);}
static bool gait_policy_to_servo_targets(const GaitPolicyLegTarget *q,uint16_t *out){return locomotion_servo_targets(q,out);}
ServoBusResult sts3215_sync_positions(ServoBus *bus,const uint8_t *ids,const uint16_t *q,size_t count) {
 (void)bus;(void)ids;assert(count==12);memcpy(commanded,q,sizeof commanded);sent++;return SERVO_BUS_OK;
}
'''+selected+'\n'+stand+'\n'+idle+'\n'+check_entry+r'''
int main(void) {
 RobotController r={0};r.locomotion_profile=locomotion_profile_id("attitudepd_v3");
 uint16_t expected[12];GaitPolicyLegTarget b[4];
 assert(check_entry(&r)==ROBOT_OK);
 assert(locomotion_targets(locomotion_profile_id("attitudepd_v2"),0,1,0,0,b));
 assert(locomotion_servo_targets(b,expected));
 assert(robot_stand(&r)==ROBOT_OK && supervised==1 && r.shared_idle);
 assert(!memcmp(commanded,expected,sizeof expected));
 robot_control_idle(&r);assert(sent==1 && !memcmp(commanded,expected,sizeof expected));
 r.locomotion_profile=locomotion_profile_id("attitudepd_v2");
 assert(robot_stand_targets(expected));
 assert(robot_stand(&r)==ROBOT_OK && !memcmp(commanded,expected,sizeof expected));
 r.shared_idle_at=0;robot_control_idle(&r);assert(sent==2 && !memcmp(commanded,expected,sizeof expected));
}
'''
    unit=tmp_path/'stand.c';unit.write_text(code);exe=tmp_path/'stand'
    subprocess.run(['clang','-O1','-I',str(INC),'-include',str(INC.parent/'tests/host_hal.h'),
        str(unit),str(INC.parent/'Src/robot_config.c'),str(INC.parent/'Src/safety.c'),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

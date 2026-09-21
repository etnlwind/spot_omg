#include "robot.h"
#include "sts3215.h"
#include "s_native_servo.h"
#include "s_native_data.h"
#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static RobotController r;static ServoBus bus;
static uint32_t tick;static unsigned writes,off,reads,transient;
static uint16_t actual[12],cmd[12];
typedef enum {NORMAL,SLOW,BLOCKED,FREE_STUCK,LOSS,TRANSIENT,WRITE_FAIL,DROOP,HOT,TILT,IMU_LOST,ABORT,INVALID,SLOW_BUS,RESIDUAL,HEALTH_GLITCH,START_EFFORT,EARLY_RESIDUAL,LOADED_SUPPORT,ENTRY,ENTRY_STOP,NEAR_BLOCKED,FAST_STAND,FAST_NATIVE_STAND} Case;
static Case scenario;
static bool direct_write;static unsigned fast_goals;
static bool retry_tilt;
static bool selected_stand(uint16_t out[12]) {
 if(scenario!=FAST_NATIVE_STAND)return robot_stand_targets(out);
 GaitPolicyLegTarget legs[4];
 for(unsigned i=0;i<4;i++)legs[i]=(GaitPolicyLegTarget){sn_standing[i][0],sn_standing[i][1],sn_standing[i][2],true};
 return s_native_servo_targets(legs,out);
}
RobotResult robot_reference_pose(RobotController *x){(void)x;return ROBOT_OK;}
uint32_t HAL_GetTick(void){return tick;}
void HAL_Delay(uint32_t n){tick+=n;assert(tick<300000);}
static bool attitude(void *ctx,int16_t *roll,int16_t *pitch){
 (void)ctx;*roll=scenario==TILT && (writes>5 || retry_tilt)?200:0;*pitch=0;
 return !(scenario==IMU_LOST && writes>5);
}
ServoBusResult sts3215_set_torque(ServoBus *b,uint8_t id,bool enabled){
 (void)b;(void)id;if(!enabled)off++;return SERVO_BUS_OK;
}
ServoBusResult sts3215_sync_positions(ServoBus *b,const uint8_t *ids,const uint16_t *pos,size_t n){
 (void)b;(void)ids;assert(n==12);writes++;
 if(scenario==WRITE_FAIL && writes>5)return SERVO_BUS_TIMEOUT;
 for(unsigned i=0;i<12;i++){
  assert(direct_write || abs((int)pos[i]-actual[i])<=120);
  cmd[i]=pos[i];
 }
 if(scenario==ABORT && writes>5)r.motion_abort_requested=true;
 return SERVO_BUS_OK;
}
ServoBusResult sts3215_sync_move(ServoBus *b,const uint8_t *ids,const uint16_t *p,size_t n,uint16_t speed,uint8_t acc){
 uint16_t stand[12];assert(selected_stand(stand));
 direct_write=(scenario==FAST_STAND || scenario==FAST_NATIVE_STAND) && memcmp(p,stand,sizeof stand)==0;
 if(direct_write){assert(speed==3400 && acc==254);fast_goals++;}
 ServoBusResult result=sts3215_sync_positions(b,ids,p,n);direct_write=false;return result;
}
ServoBusResult sts3215_read_state(ServoBus *b,uint8_t id,Sts3215State *s){
 (void)b;tick+=scenario==SLOW_BUS?60:8;reads++;
 if(scenario==LOSS && writes>5)return SERVO_BUS_TIMEOUT;
 if(scenario==TRANSIENT && writes>5 && transient++<2)return SERVO_BUS_TIMEOUT;
 unsigned i=id-1;memset(s,0,sizeof(*s));
 int delta=(int)cmd[i]-actual[i];int limit=(scenario==SLOW || scenario==EARLY_RESIDUAL)?1:8;
 if(delta>limit)delta=limit;
 if(delta< -limit)delta=-limit;
 if(scenario==RESIDUAL || scenario==LOADED_SUPPORT || (scenario==EARLY_RESIDUAL && i==2)) {
  int error=(int)cmd[i]-actual[i];
  int dead=scenario==LOADED_SUPPORT?67:26;
  if(abs(error)<=dead)delta=0;
  else if(abs(delta)>abs(error)-dead)delta=error>0?abs(error)-dead:-(abs(error)-dead);
 }
 if(scenario==START_EFFORT && abs((int)cmd[i]-actual[i])<50)delta=0;
 if(!((scenario==BLOCKED || scenario==FREE_STUCK || scenario==NEAR_BLOCKED) && i==2))actual[i]+=delta;
 s->position=(scenario==INVALID && writes>5)?65535:actual[i];
 s->load=(scenario==BLOCKED || scenario==NEAR_BLOCKED) && i==2?800:150;
 if(scenario==LOADED_SUPPORT)s->load=450;
 s->current=100;s->voltage_mv=scenario==DROOP && writes>5?9900:11800;
 s->temperature_c=scenario==HOT && writes>5?80:35;
 if(scenario==HEALTH_GLITCH && writes>5 && transient++==0)s->temperature_c=150;
 return SERVO_BUS_OK;
}
static RobotResult run(Case c){
 memset(&r,0,sizeof(r));memset(&bus,0,sizeof(bus));r.bus=&bus;
 safety_init(&r.safety,NULL);r.profile_speed=3400;r.profile_acceleration=254;
 r.attitude_reader=attitude;scenario=c;tick=writes=reads=off=transient=0;
 if(retry_tilt){robot_latch_locomotion_fault(&r,ROBOT_TILT_LIMIT);assert(robot_prepare_new_command(&r)==ROBOT_OK);}
 assert((c==FAST_STAND || c==FAST_NATIVE_STAND)?robot_landing_targets(actual):robot_straight_targets(actual));
 fast_goals=0;memcpy(cmd,actual,sizeof(cmd));
 uint16_t to[12];assert(selected_stand(to));
 if(scenario==EARLY_RESIDUAL)to[2]=actual[2]-30;
 if(scenario==NEAR_BLOCKED)to[2]=actual[2]-80;
 if(c==ENTRY || c==ENTRY_STOP){r.drive_active=true;r.drive_pose_entry=true;r.drive_stop_requested=c==ENTRY_STOP;}
 RobotResult result=robot_supervised_pose(&r,to,true);assert(off==0);return result;
}
static void test_support_windows(void) {
 SupportWindow w={0};
 assert(support_update(&w,0,2000,1933,0,0,650,100,11300,40,0)==SUPPORT_OBSERVING);
 for(unsigned t=200;t<1200;t+=200)support_update(&w,t,2000,1933,0,0,650,100,11300,40,0);
 assert(support_update(&w,1200,2000,1934,0,0,650,100,11300,40,0)==SUPPORT_STABLE);
 for(unsigned t=1400;t<2400;t+=200)support_update(&w,t,2000,1934,0,0,650,100,11300,40,0);
 assert(support_update(&w,2400,2000,1910,0,0,650,100,11300,40,0)==SUPPORT_DRIFT);
 memset(&w,0,sizeof w);
 assert(support_update(&w,0,2000,1933,0,0,650,750,11300,40,0)==SUPPORT_EFFORT);
 assert(support_update(&w,0,2000,1933,0,0,650,100,10000,40,0)==SUPPORT_VOLTAGE);
 assert(support_update(&w,0,2000,1933,0,0,650,100,11300,80,0)==SUPPORT_HEALTH);
 support_update(&w,0,2000,1933,0,0,400,100,11300,40,0);
 for(unsigned t=200;t<1200;t+=200)support_update(&w,t,2000,1933,0,0,400,100,11300,40,0);
 assert(support_update(&w,1200,2020,1953,0,0,400,100,11300,40,0)==SUPPORT_ADJUSTING);
 assert(support_update(&w,3000,2020,1953,0,0,400,100,11300,40,0)==SUPPORT_OBSERVING);
}
int main(void){
 test_support_windows();
 assert(run(FAST_STAND)==ROBOT_OK);assert(fast_goals==1 && r.pose_diagnostics.nominal_ms==0);
 assert(run(FAST_NATIVE_STAND)==ROBOT_OK);assert(fast_goals==1 && r.pose_diagnostics.nominal_ms==0);
 assert(run(NORMAL)==ROBOT_OK);assert(r.pose_diagnostics.reason==POSE_COMPLETE);
 assert(run(SLOW)==ROBOT_OK);assert(r.pose_diagnostics.elapsed_ms>r.pose_diagnostics.nominal_ms+2000);
 assert(r.pose_diagnostics.waits>0);
 assert(run(BLOCKED)==ROBOT_VERIFY_ERROR);assert(r.pose_diagnostics.reason==POSE_OBSTRUCTION_SUSPECTED);
 assert(r.pose_diagnostics.hold_verified==2);
 assert(run(FREE_STUCK)==ROBOT_VERIFY_ERROR);assert(r.pose_diagnostics.reason==POSE_NO_PROGRESS);
 assert(run(LOSS)==ROBOT_BUS_ERROR);assert(r.pose_diagnostics.reason==POSE_FEEDBACK_LOST && r.pose_diagnostics.hold_verified==3);
 assert(run(TRANSIENT)==ROBOT_OK);assert(r.pose_diagnostics.retries==2);
 assert(run(WRITE_FAIL)==ROBOT_BUS_ERROR);assert(r.pose_diagnostics.hold_verified==3);
 assert(run(DROOP)==ROBOT_SAFETY_FAULT);assert(r.pose_diagnostics.reason==POSE_LOW_VOLTAGE);
 assert(run(HOT)==ROBOT_SAFETY_FAULT);assert(r.pose_diagnostics.reason==POSE_SERVO_FAULT);
 assert(run(TILT)==ROBOT_TILT_LIMIT);assert(run(IMU_LOST)==ROBOT_IMU_ERROR);
 retry_tilt=true;assert(run(TILT)==ROBOT_OK);assert(writes>5);retry_tilt=false;
 assert(run(ABORT)==ROBOT_MOTION_ABORTED);assert(run(INVALID)==ROBOT_POSITION_LIMIT);
 assert(run(SLOW_BUS)==ROBOT_BUS_ERROR);assert(writes==0);
 assert(run(RESIDUAL)==ROBOT_OK);assert(r.pose_diagnostics.reason==POSE_COMPLETE_RESIDUAL);
 for(unsigned i=0;i<12;i++)assert(abs((int)cmd[i]-actual[i])<=40);
 assert(run(HEALTH_GLITCH)==ROBOT_OK);assert(r.pose_diagnostics.health_recovered==1);
 assert(r.pose_diagnostics.suspect_temperature==150);
 assert(run(START_EFFORT)==ROBOT_OK);
 assert(r.pose_diagnostics.reason==POSE_LOAD_SUPPORTED);
 assert(r.pose_diagnostics.effort_probes>0);
 /* The modeled servo needs 50 ticks of sustained error; it makes progress
  * with the probe but is not falsely declared arrived with 50 ticks left. */
 assert(r.pose_diagnostics.elapsed_ms>2000);
 assert(run(EARLY_RESIDUAL)==ROBOT_OK);
 assert(r.pose_diagnostics.reason==POSE_COMPLETE_RESIDUAL);
 assert(run(LOADED_SUPPORT)==ROBOT_OK);assert(r.pose_diagnostics.reason==POSE_LOAD_SUPPORTED);
 assert(run(NEAR_BLOCKED)==ROBOT_VERIFY_ERROR);
 assert(run(ENTRY)==ROBOT_OK);assert(run(ENTRY_STOP)==ROBOT_MOTION_ABORTED);
 assert(run(NORMAL)==ROBOT_OK);robot_support_reset(&r);r.shared_idle=true;
 for(unsigned n=0;n<200;n++){assert(robot_support_observe(&r,cmd));HAL_Delay(20);}
 assert(r.support_decision==SUPPORT_STABLE && r.support_stable_mask==0xfff);
 unsigned off_before=off;scenario=DROOP;
 assert(!robot_support_observe(&r,cmd));
 assert(!r.shared_idle && r.support_decision==SUPPORT_VOLTAGE && off==off_before);
 r.drive_active=true;r.drive_pose_entry=false;uint16_t rejected[12];assert(robot_stand_targets(rejected));
 assert(robot_supervised_pose(&r,rejected,false)==ROBOT_INVALID_ARGUMENT);
 puts("pose supervisor: 16 scenarios passed; no torque-off; slow progress completes");
}

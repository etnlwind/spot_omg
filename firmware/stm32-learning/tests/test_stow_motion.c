#include "robot.h"
#include "stow_control.h"
#include "sts3215.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
static RobotController r;
static ServoBus fake_bus;
static uint32_t now;
static int32_t actual[12];
static uint8_t memory[12][80];
static bool hold_goal[12];
static bool modulo_feedback;
static int relax_calls;
static int writes,abort_at,bad_id,refuse_settings,tracking_error,check_unfold,landing_calls,hold_drift;
uint32_t HAL_GetTick(void){return now;}
void HAL_Delay(uint32_t t){
 now+=t;
 for(unsigned i=0;i<12;i++)if(hold_goal[i] && memory[i][40]) {
  int32_t target=stow_unwire(memory[i][42]|memory[i][43]<<8);
  int32_t delta=target-actual[i],step=(int32_t)(t/10);if(step<1)step=1;
  if(delta>step)delta=step;if(delta< -step)delta= -step;actual[i]+=delta;
 }
}
ServoBusResult servo_bus_read(ServoBus*b,uint8_t id,uint8_t addr,uint8_t*d,size_t n){
 (void)b;if(id==bad_id)return SERVO_BUS_TIMEOUT;memcpy(d,memory[id-1]+addr,n);return SERVO_BUS_OK;
}
ServoBusResult servo_bus_write(ServoBus*b,uint8_t id,uint8_t addr,const uint8_t*d,size_t n){
 (void)b;if(id==bad_id)return SERVO_BUS_TIMEOUT;
 if(addr==9 && refuse_settings)return SERVO_BUS_OK;
 memcpy(memory[id-1]+addr,d,n);if(addr==41)hold_goal[id-1]=true;return SERVO_BUS_OK;
}
ServoBusResult sts3215_set_torque(ServoBus*b,uint8_t id,bool on){(void)b;memory[id-1][40]=on;return SERVO_BUS_OK;}
ServoBusResult sts3215_read_state_raw(ServoBus*b,uint8_t id,Sts3215State*s){
 (void)b;if(id==bad_id)return SERVO_BUS_TIMEOUT;
 int32_t position=actual[id-1];if(modulo_feedback && (id==2 || id==5))position=(position%4096+4096)%4096;
 memset(s,0,sizeof(*s));s->position=stow_wire(position+(tracking_error && writes>1 && id==2?600:0)+(hold_drift && memory[id-1][40]?20:0));s->temperature_c=30;return SERVO_BUS_OK;
}
ServoBusResult sts3215_read_state(ServoBus*b,uint8_t id,Sts3215State*s){
 ServoBusResult result=sts3215_read_state_raw(b,id,s);
 if(result==SERVO_BUS_OK && (id==2 || id==5)) {
   int32_t raw=stow_unwire(s->position);s->position=(raw%4096+4096)%4096;
   if(!b->front_origin_valid[id==2?0:1])b->front_position_bias[id==2?0:1]=raw-s->position;
 }
 return result;
}
ServoBusResult servo_bus_sync_write(ServoBus*b,uint8_t addr,uint8_t size,const uint8_t*ids,const uint8_t*items,size_t n){
 (void)b;assert(addr==41 && size==7 && n==12);writes++;
 for(unsigned i=0;i<12;i++) {
  int32_t next=stow_unwire(items[7*i+1]|items[7*i+2]<<8);
  assert(abs(next-actual[ids[i]-1])<30);
  if(check_unfold && ids[i]==2)assert(next<=actual[1]);
  if(check_unfold && ids[i]==5)assert(next>=actual[4]);
  actual[ids[i]-1]=next;hold_goal[ids[i]-1]=false;
 }
 if(writes==abort_at)r.motion_abort_requested=true;
 return SERVO_BUS_OK;
}
RobotResult robot_relax(RobotController*x){(void)x;relax_calls++;for(int i=0;i<12;i++)memory[i][40]=0;return ROBOT_OK;}
RobotResult robot_hold(RobotController*x){(void)x;for(int i=0;i<12;i++)memory[i][40]=1;return ROBOT_OK;}
RobotResult robot_landing(RobotController*x){(void)x;landing_calls++;for(unsigned i=0;i<12;i++)assert(stow_encode(i,stow_endpoint(i,false),actual+i));return ROBOT_OK;}
static bool imu(void*c,int16_t*roll,int16_t*pitch){(void)c;*roll=*pitch=0;return true;}
static void reset(void){memset(&fake_bus,0,sizeof fake_bus);memset(&r,0,sizeof r);memset(memory,0,sizeof memory);safety_init(&r.safety,NULL);r.attitude_reader=imu;r.bus=&fake_bus;now=writes=abort_at=bad_id=refuse_settings=0;memset(hold_goal,0,sizeof hold_goal);modulo_feedback=false;relax_calls=0;
 for(int i=0;i<12;i++){memory[i][30]=1;memory[i][11]=255;memory[i][12]=15;}
 robot_landing(&r);
}
int main(void){
 reset();assert(robot_stow_hold_check(&r,2,true,false)==ROBOT_OK);
 assert(memory[1][11]==255 && memory[1][12]==15 && memory[1][40]==0);
 assert(writes==0);
 reset();hold_drift=1;
 assert(robot_stow_hold_check(&r,2,true,false)==ROBOT_VERIFY_ERROR);
 assert(r.stow_failure_actual-r.stow_failure_target==20 && r.stow_failure_elapsed==10);
 assert(memory[1][11]==255 && memory[1][12]==15 && memory[1][40]==0);
 hold_drift=0;
 reset();hold_drift=1;
 assert(robot_stow(&r,true)==ROBOT_VERIFY_ERROR);
 assert(writes==0 && !fake_bus.front_origin_valid[0]);
 for(int i=0;i<12;i++)assert(memory[i][40]==0);
 hold_drift=0;
 reset();memory[0][40]=1;assert(robot_stow_hold_check(&r,2,true,false)==ROBOT_INVALID_ARGUMENT);
 assert(memory[1][11]==255 && memory[1][40]==0);
 reset();modulo_feedback=true;actual[1]=4956;
 assert(robot_stow_hold_check(&r,2,true,true)==ROBOT_OK);
 assert((memory[1][42]|memory[1][43]<<8)==4956);
 reset();modulo_feedback=true;actual[4]=-938;
 assert(robot_stow_hold_check(&r,5,true,true)==ROBOT_OK);
 assert(stow_unwire(memory[4][42]|memory[4][43]<<8)==-938);
 reset();assert(robot_stow_probe(&r)==ROBOT_OK && !r.stow_active);
 for(unsigned i=0;i<12;i++)assert(stow_encode(i,i%3==0?0:(i%3==1?45:90),actual+i));
 assert(robot_stow_probe(&r)==ROBOT_OK && !r.stow_active);
 /* Actual hand-folded readback, off the nominal interpolation line. */
 const int32_t manual[12]={1926,858,2369,2091,3161,1662,2105,2708,2408,1949,1469,1610};
 memcpy(actual,manual,sizeof actual);landing_calls=0;
 assert(robot_stow_probe(&r)==ROBOT_OK && r.stow_active && !r.stow_complete);
 assert(writes==0);
 check_unfold=1;
 assert(robot_stow(&r,false)==ROBOT_OK && !r.stow_active);
 assert(landing_calls==0); /* No ordinary pose move / modulo shortcut. */
 assert(fake_bus.front_position_bias[0]==-4096 && fake_bus.front_position_bias[1]==4096);
 check_unfold=0;
 reset();tracking_error=1;assert(robot_stow(&r,true)==ROBOT_VERIFY_ERROR);
 assert(r.stow_tracking_failure && r.last_failed_servo_id==2);
 assert(r.stow_failure_actual-r.stow_failure_target==600);
 assert(r.stow_failure_elapsed==40);
 for(int i=0;i<12;i++)assert(!memory[i][40]);
 tracking_error=0;
 reset();assert(robot_stow(&r,true)==ROBOT_OK);assert(r.stow_active && r.stow_complete);
 assert(actual[1]>4095 && actual[4]<0);
 for(int i=0;i<12;i++)assert(!memory[i][40]);
 int released_before_unfold=relax_calls;
 assert(robot_stow(&r,false)==ROBOT_OK);assert(!r.stow_active);
 assert(relax_calls==released_before_unfold);
 for(int i=0;i<12;i++)assert(memory[i][40]==1);
 assert(memory[1][11]==0 && memory[1][12]==0);
 for(int i=0;i<12;i++)assert(actual[i]>=0 && actual[i]<=4095);
 reset();abort_at=300;assert(robot_stow(&r,true)==ROBOT_MOTION_ABORTED);assert(r.stow_active && !r.stow_complete);
 abort_at=0;assert(robot_stow(&r,true)==ROBOT_OK);assert(robot_stow(&r,false)==ROBOT_OK);
 reset();refuse_settings=1;assert(robot_stow(&r,true)==ROBOT_CONFIG_ERROR);assert(writes==0);
 reset();bad_id=5;assert(robot_stow(&r,true)==ROBOT_BUS_ERROR);assert(writes==0);
 reset();assert(robot_stow(&r,true)==ROBOT_OK);r.stow_active=false;r.stow_complete=false;
 actual[1]%=4096;actual[4]=(actual[4]%4096+4096)%4096;
 assert(robot_stow_probe(&r)==ROBOT_OK && r.stow_active);
 /* Unfold follows measured geometry and preserves the new continuous origin. */
 assert(robot_stow(&r,false)==ROBOT_OK && !r.stow_active);
 assert(fake_bus.front_position_bias[0]==-4096 && fake_bus.front_position_bias[1]==4096);
 /* Real STS3250 modulo feedback, preserved internal turns: full boundary cycle. */
 reset();modulo_feedback=true;
 assert(robot_stow(&r,true)==ROBOT_OK);
 assert(actual[1]>4095 && actual[4]<0);
 assert(fake_bus.front_stow_origin[0]==0 && fake_bus.front_stow_origin[1]==0);
 assert(robot_stow_hold(&r)==ROBOT_OK);
 assert(robot_stow(&r,false)==ROBOT_OK);
 assert(fake_bus.front_position_bias[0]==0 && fake_bus.front_position_bias[1]==0);
 /* Reset STM32 cache while servo's internal turns remain beyond one turn. */
 assert(robot_stow(&r,true)==ROBOT_OK);
 r.stow_active=false;memset(&fake_bus,0,sizeof fake_bus);
 assert(robot_stow_probe(&r)==ROBOT_OK && r.stow_active);
 assert(robot_stow(&r,false)==ROBOT_OK);
 puts("stow transport, signed boundary, resume, rejection, reboot detection passed");return 0;
}

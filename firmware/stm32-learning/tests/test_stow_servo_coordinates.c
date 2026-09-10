#include "sts3215.h"
#include "stow_control.h"
#include "pose_control.h"
#include "feetech_protocol.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static uint16_t sensed,goal;
ServoBusResult servo_bus_ping(ServoBus*b,uint8_t id){(void)b;(void)id;return SERVO_BUS_OK;}
ServoBusResult servo_bus_read(ServoBus*b,uint8_t id,uint8_t address,uint8_t*data,size_t n){
 (void)b;(void)id;assert(address==56);memset(data,0,n);feetech_encode_u16(sensed,data);return SERVO_BUS_OK;
}
ServoBusResult servo_bus_write(ServoBus*b,uint8_t id,uint8_t address,const uint8_t*data,size_t n){
 (void)b;(void)id;(void)n;if(address==41)goal=feetech_decode_u16(data+1);return SERVO_BUS_OK;
}
ServoBusResult servo_bus_sync_write(ServoBus*b,uint8_t address,uint8_t size,const uint8_t*ids,const uint8_t*items,size_t n){
 (void)b;(void)size;(void)ids;assert(n==1);goal=feetech_decode_u16(items+(address==41?1:0));return SERVO_BUS_OK;
}
int main(void){
 uint16_t from[12]={0},to[12],previous[12]={0},frame[12];
 for(unsigned i=0;i<12;i++)to[i]=455;
 unsigned duration=pose_duration(from,to);assert(duration>=5400);
 for(unsigned elapsed=20;elapsed<=duration;elapsed+=20){
  pose_frame(from,to,duration,elapsed,frame);
  for(unsigned i=0;i<12;i++){assert(frame[i]>=previous[i] && frame[i]-previous[i]<=4);previous[i]=frame[i];}
 }
 assert(frame[0]==455 && pose_duration(to,to)==0);
 uint16_t landing[12],stand[12],straight[12];
 assert(robot_landing_targets(landing) && robot_stand_targets(stand) && robot_straight_targets(straight));
 assert(pose_duration(landing,stand)==0);
 assert(pose_duration(stand,landing)>=5400);
 assert(pose_duration(stand,straight)>0 && pose_duration(straight,stand)>0);
 ServoBus b={0};Sts3215State s;
 for(int32_t raw=-32767;raw<=32767;raw++)assert(stow_unwire(stow_wire(raw))==raw);
 for(unsigned id=2;id<=5;id+=3)for(int turns=-2;turns<=2;turns++) {
  int raw=1900+4096*turns;sensed=stow_wire(raw);
  assert(sts3215_read_state_raw(&b,id,&s)==SERVO_BUS_OK && s.position==sensed);
  assert(sts3215_read_state(&b,id,&s)==SERVO_BUS_OK && s.position==1900);
  uint16_t target=1905;uint8_t axis=id;
  assert(sts3215_sync_positions(&b,&axis,&target,1)==SERVO_BUS_OK && stow_unwire(goal)==raw+5);
  assert(sts3215_sync_move(&b,&axis,&target,1,800,30)==SERVO_BUS_OK && stow_unwire(goal)==raw+5);
  assert(sts3215_write_position(&b,id,target,800,30)==SERVO_BUS_OK && stow_unwire(goal)==raw+5);
 }
 /* Modulo readback must not erase a verified command turn after Stow. */
 b.front_origin_valid[0]=true;b.front_position_bias[0]=4096;sensed=861;
 assert(sts3215_read_state(&b,2,&s)==SERVO_BUS_OK && s.position==861);
 assert(b.front_position_bias[0]==4096);
 assert(sts3215_write_position(&b,2,861,40,10)==SERVO_BUS_OK && stow_unwire(goal)==4957);
 b.front_origin_valid[1]=true;b.front_position_bias[1]=-4096;sensed=3159;
 assert(sts3215_read_state(&b,5,&s)==SERVO_BUS_OK && s.position==3159);
 assert(sts3215_write_position(&b,5,3159,40,10)==SERVO_BUS_OK && stow_unwire(goal)==-937);
 assert(stow_feedback_error(1,4097)==0 && stow_feedback_error(4095,-1)==0);
 assert(stow_feedback_error(3,4097)==2);
 uint16_t target=5000;uint8_t axis=2;
 assert(sts3215_sync_positions(&b,&axis,&target,1)==SERVO_BUS_INVALID_ARGUMENT);
 puts("signed wire and ordinary gait origin continuity passed");return 0;
}

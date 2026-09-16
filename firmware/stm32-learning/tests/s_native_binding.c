#include "s_native.h"
#include "s_native_impl.h"
#include "locomotion_servo.h"
#include "s_native_servo.h"
static SNativeControl state;
void reset(void) { s_native_reset(&state); }
void reset_v624(void) { s_native_reset_v624(&state); }
void reset_v625(void) { s_native_reset_v625(&state); }
void reset_v627(void) { s_native_reset_v627(&state); }
void reset_v626(void) { s_native_reset_v626(&state); }
void reset_v623(void) { s_native_reset_v623(&state); }
void reset_v621(void) { s_native_reset_profile(&state,true); }
int step(float phase,float amplitude,float linear,float yaw,float rl,float ry,float dt,int stop,float out[12]) {
    GaitPolicyLegTarget target[4];
    if(!s_native_step(&state,phase,amplitude,linear,yaw,rl,ry,dt,stop!=0,target))return 0;
    uint16_t ticks[12];
    if(!s_native_servo_targets(target,ticks))return 0;
    for(int i=0;i<4;i++){out[3*i]=target[i].j1_deg;out[3*i+1]=target[i].j2_deg;out[3*i+2]=target[i].j3_deg;}
    return 1;
}
int stopped(void) { return s_native_stopped(&state); }
int encode(const float cad[12],uint16_t ticks[12]) {
    GaitPolicyLegTarget target[4];
    for(int i=0;i<4;i++)target[i]=(GaitPolicyLegTarget){cad[3*i],cad[3*i+1],cad[3*i+2],true};
    return s_native_servo_targets(target,ticks);
}

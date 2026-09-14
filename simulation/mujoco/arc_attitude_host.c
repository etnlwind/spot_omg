/* Standalone opt-in host harness; production export may use the same wrapper. */
#include "arc_attitude.h"

int spot_arc_attitude(float state[4],const float values[12],float phase,float period,float duty,
        const float imu[4],int available,float dt,const float config[6],float output[12]) {
    GaitPolicyLegTarget nominal[4],result[4];ArcAttitudeState control;
    for(int i=0;i<4;i++){
        control.dz[i]=state[i];
        nominal[i]=(GaitPolicyLegTarget){values[3*i],values[3*i+1],values[3*i+2],false};
    }
    ArcAttitudeConfig parameters={config[0],config[1],config[2],config[3],config[4],config[5]};
    if(!arc_attitude_apply(&control,nominal,phase,period,duty,imu[0],imu[1],imu[2],imu[3],
        available!=0,dt,&parameters,result))return 0;
    for(int i=0;i<4;i++){
        state[i]=control.dz[i];
        output[3*i]=result[i].j1_deg;output[3*i+1]=result[i].j2_deg;output[3*i+2]=result[i].j3_deg;
    }
    return 1;
}

void spot_arc_attitude_foot(const float values[12],float output[12]) {
    for(int i=0;i<4;i++)arc_foot(i,values+3*i,output+3*i,NULL);
}

int spot_arc_attitude_targets(float phase,float yaw,float values[12]) {
    GaitPolicyLegTarget pose[4];
    if(!arc_turn_targets_configured(phase,1,0,yaw,.02f,.5f,0,0,pose))return 0;
    for(int i=0;i<4;i++){
        values[3*i]=pose[i].j1_deg;values[3*i+1]=pose[i].j2_deg;values[3*i+2]=pose[i].j3_deg;
    }
    return 1;
}

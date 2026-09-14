#ifndef ARC_TRANSFER_H
#define ARC_TRANSFER_H
#include "arc_preload.h"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "fp-contract=off")
#endif
/* Experimental scheduled gravity feedforward, not a contact observer or MPC.
 * phase is the target's ALREADY advanced phase. Only extra_lead_s is added.
 * The same lowest cushion vertex defines each load lever and its Jacobian.
 * This estimate is not a claim that both scheduled feet have touched ground. */
static inline float arc_transfer_smooth(float x) {
 x=gait_policy_clampf(x,0.f,1.f);return x*x*x*(10.f+x*(-15.f+6.f*x));
}
static inline float arc_transfer_support(float phase,float duty,float width) {
 if(width<=0.f){phase-=floorf(phase);return phase<duty?1.f:0.f;}
 float u=phase+.5f*width;u-=floorf(u);
 if(u<width)return arc_transfer_smooth(u/width);
 if(u<duty)return 1.f;
 if(u<duty+width)return 1.f-arc_transfer_smooth((u-duty)/width);
 return 0.f;
}
static inline bool arc_transfer_apply(float state[12],GaitPolicyLegTarget pose[4],
 float phase,float period,float duty,float width_s,float extra_lead_s,
 float gain,float stiffness,float dt) {
 if(!isfinite(phase)||!isfinite(period)||period<=0.f||!isfinite(duty)||duty<.5f||duty>.65f||
    !isfinite(width_s)||width_s<0.f||width_s>period*(1.f-duty)||
    !isfinite(extra_lead_s)||fabsf(extra_lead_s)>period*.25f||
    !isfinite(gain)||gain<0.f||gain>1.f||!isfinite(stiffness)||stiffness<=0.f||
    !isfinite(dt)||dt<=0.f||dt>.060f)return false;
 for(int i=0;i<4;i++)if(!isfinite(pose[i].j1_deg)||!isfinite(pose[i].j2_deg)||!isfinite(pose[i].j3_deg))return false;
 for(int i=0;i<12;i++)if(!isfinite(state[i])||fabsf(state[i])>4.f)return false;
 float bias[4][3],feet[4][3],jz[4][3],com[3],shares[4]={0};
 arc_gravity_model(pose,bias,feet,jz,com);
 for(int i=0;i<4;i++){
  float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg},point[3];
  arc_foot_geometry(i,q,point,NULL,feet[i]);
 }
 float planned=phase+extra_lead_s/period;
 float wa=arc_transfer_support(planned,duty,width_s/period);
 float wb=arc_transfer_support(planned+.5f,duty,width_s/period);
 float total=wa+wb;if(!isfinite(total)||total<1.e-6f)return false;
 const int pairs[2][2]={{0,3},{1,2}};
 for(int pair=0;pair<2;pair++){
  int a=pairs[pair][0],b=pairs[pair][1];
  float dx=feet[b][0]-feet[a][0],dy=feet[b][1]-feet[a][1];
  float f=gait_policy_clampf(((com[0]-feet[a][0])*dx+(com[1]-feet[a][1])*dy)/fmaxf(1.e-9f,dx*dx+dy*dy),.1f,.9f);
  float weight=(pair==0?wa:wb)/total;shares[a]=(1.f-f)*weight;shares[b]=f*weight;
 }
 float next_state[12];GaitPolicyLegTarget next_pose[4];
 for(int i=0;i<4;i++){
  float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg};
  for(int j=0;j<3;j++){
   int k=3*i+j;float torque=bias[i][j]-jz[i][j]*arc_total_mass*9.81f*shares[i];
   float requested=gait_policy_clampf(gain*torque/stiffness*180.f/GAIT_POLICY_PI,-4.f,4.f);
   next_state[k]=state[k]+gait_policy_clampf(requested-state[k],-12.5f*dt,12.5f*dt);
   q[j]+=next_state[k];if(!isfinite(q[j]))return false;
  }
  if(q[0]<-30.f||q[0]>30.f||q[1]<-45.f||q[1]>100.f||q[2]<0.f||q[2]>150.f)return false;
  next_pose[i]=pose[i];next_pose[i].j1_deg=q[0];next_pose[i].j2_deg=q[1];next_pose[i].j3_deg=q[2];
 }
 for(int i=0;i<12;i++)state[i]=next_state[i];
 for(int i=0;i<4;i++)pose[i]=next_pose[i];
 return true;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

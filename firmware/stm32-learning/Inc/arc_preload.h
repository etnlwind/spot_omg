#ifndef ARC_PRELOAD_H
#define ARC_PRELOAD_H
#include "arc_turn.h"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "fp-contract=off")
#endif
/* Static inverse-gravity estimate, position servo feedforward. Not torque MPC.
 * Geometry and mass are fixed CAD estimates, never simulator contact forces. */
static inline void arc_gravity_model(const GaitPolicyLegTarget pose[4],float bias[4][3],float foot[4][3],float foot_jz[4][3],float com[3]) {
 for(int k=0;k<3;k++)com[k]=arc_fixed_moment[k];
 for(int i=0;i<4;i++){
  float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg};
  float centers[3][3],jac[3][3];
  arc_foot(i,q,foot[i],jac);
  for(int j=0;j<3;j++){
   foot_jz[i][j]=jac[2][j];
   arc_transform(i,q,j+1,arc_link_com[i][j],centers[j],false);
   for(int k=0;k<3;k++)com[k]+=arc_link_mass[i][j]*centers[j][k];
  }
  for(int j=0;j<3;j++){
   float axis[3],origin[3];bias[i][j]=0;
   arc_transform(i,q,j,arc_axes[i][j],axis,true);
   arc_transform(i,q,j,arc_anchors[i][j],origin,false);
   for(int link=j;link<3;link++){
    float v[3],cross[3];for(int k=0;k<3;k++)v[k]=centers[link][k]-origin[k];
    arc_cross(axis,v,cross);bias[i][j]+=arc_link_mass[i][link]*9.81f*cross[2];
   }
  }
 }
 for(int k=0;k<3;k++)com[k]/=arc_total_mass;
}
static inline bool arc_preload_apply(float state[12],GaitPolicyLegTarget pose[4],float phase,float duty,float gain,float stiffness) {
 if(!isfinite(gain)||gain<0||gain>1||!isfinite(stiffness)||stiffness<=0||!isfinite(phase)||!isfinite(duty)||duty<.5f||duty>.65f)return false;
 for(int i=0;i<4;i++)if(!isfinite(pose[i].j1_deg)||!isfinite(pose[i].j2_deg)||!isfinite(pose[i].j3_deg))return false;
 for(int k=0;k<12;k++)if(!isfinite(state[k]))return false;
 float next_state[12];GaitPolicyLegTarget next_pose[4];
 float bias[4][3],feet[4][3],jz[4][3],com[3],share[4]={0};int ids[4],n=0;
 arc_gravity_model(pose,bias,feet,jz,com);
 for(int i=0;i<4;i++){
  float u=phase+((i==1||i==2)?.5f:0);u-=floorf(u);
  if(u<duty)ids[n++]=i;
 }
 if(!n)return false;
 for(int a=0;a<n;a++)share[ids[a]]=1.f/n;
 if(n==2){
  int a=ids[0],b=ids[1];float dx=feet[b][0]-feet[a][0],dy=feet[b][1]-feet[a][1];
  float f=gait_policy_clampf(((com[0]-feet[a][0])*dx+(com[1]-feet[a][1])*dy)/fmaxf(1.e-9f,dx*dx+dy*dy),.1f,.9f);
  share[a]=1-f;share[b]=f;
 }
 for(int i=0;i<4;i++){
  float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg};
  for(int j=0;j<3;j++){
   float torque=bias[i][j]-jz[i][j]*arc_total_mass*9.81f*share[i];
   float requested=gait_policy_clampf(gain*torque/stiffness*180.f/GAIT_POLICY_PI,-4,4);
   int k=i*3+j;next_state[k]=state[k]+gait_policy_clampf(requested-state[k],-.25f,.25f);q[j]+=next_state[k];
  }
  if(q[0]<-30||q[0]>30||q[1]<-45||q[1]>100||q[2]<0||q[2]>150)return false;
  next_pose[i]=pose[i];next_pose[i].j1_deg=q[0];next_pose[i].j2_deg=q[1];next_pose[i].j3_deg=q[2];
 }
 for(int k=0;k<12;k++)state[k]=next_state[k];
 for(int i=0;i<4;i++)pose[i]=next_pose[i];
 return true;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

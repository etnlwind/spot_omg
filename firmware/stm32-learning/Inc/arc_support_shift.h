#ifndef ARC_SUPPORT_SHIFT_H
#define ARC_SUPPORT_SHIFT_H
#include "arc_preload.h"
/* Scheduled, bounded COM translation toward the upcoming cushion support line.
 * No measured contact or plant state. Experimental until dynamic validation. */
static inline bool arc_support_shift_feedback(float state[2],GaitPolicyLegTarget pose[4],float phase,float width,float gain,
 float roll,float pitch,float roll_rate,float pitch_rate,float feedback_gain,float derivative_s) {
 if(!isfinite(phase)||!isfinite(width)||width<.001f||width>.4f||!isfinite(gain)||gain<0||gain>1.5f)return false;
 if(!isfinite(roll)||!isfinite(pitch)||!isfinite(roll_rate)||!isfinite(pitch_rate)||!isfinite(feedback_gain)||feedback_gain<0||feedback_gain>.5f||!isfinite(derivative_s)||derivative_s<0||derivative_s>.1f)return false;
 float bias[4][3],feet[4][3],jz[4][3],com[3];
 arc_gravity_model(pose,bias,feet,jz,com);
 for(int i=0;i<4;i++){
  float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg},point[3];
  arc_foot_geometry(i,q,point,NULL,feet[i]);
 }
 float u=phase+width*.5f;u-=floorf(u);float w;
 if(u<width)w=gait_policy_smootherstep(u/width);
 else if(u<.5f)w=1;
 else if(u<.5f+width)w=1-gait_policy_smootherstep((u-.5f)/width);
 else w=0;
 const int pairs[2][2]={{0,3},{1,2}};float shift[2]={0};
 for(int p=0;p<2;p++){
  int a=pairs[p][0],b=pairs[p][1];float dx=feet[b][0]-feet[a][0],dy=feet[b][1]-feet[a][1];
  float f=gait_policy_clampf(((com[0]-feet[a][0])*dx+(com[1]-feet[a][1])*dy)/fmaxf(1.e-9f,dx*dx+dy*dy),.1f,.9f);
  float weight=p==0?w:1-w;
  for(int k=0;k<2;k++)shift[k]+=weight*(feet[a][k]+f*(feet[b][k]-feet[a][k])-com[k])*gain;
  float length=sqrtf(dx*dx+dy*dy),tx=dx/fmaxf(length,1.e-6f),ty=dy/fmaxf(length,1.e-6f);
  float free_tilt=tx*(roll+derivative_s*roll_rate)+ty*(pitch+derivative_s*pitch_rate);
  shift[0]+=weight*(-ty)*free_tilt*feedback_gain;
  shift[1]+=weight*tx*free_tilt*feedback_gain;
 }
 float norm=sqrtf(shift[0]*shift[0]+shift[1]*shift[1]);
 if(norm>.01f)for(int k=0;k<2;k++)shift[k]*=.01f/norm;
 float next[2];for(int k=0;k<2;k++)next[k]=state[k]+gait_policy_clampf(shift[k]-state[k],-.001f,.001f);
 GaitPolicyLegTarget result[4];
 for(int i=0;i<4;i++){
  float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg},point[3];arc_foot(i,q,point,NULL);
  for(int k=0;k<2;k++)point[k]-=next[k];
  if(!arc_ik(i,point,q))return false;
  result[i]=(GaitPolicyLegTarget){q[0],q[1],q[2],pose[i].stance};
 }
 for(int i=0;i<4;i++)pose[i]=result[i];for(int k=0;k<2;k++)state[k]=next[k];
 return true;
}
static inline bool arc_support_shift(float state[2],GaitPolicyLegTarget pose[4],float phase,float width,float gain) {
 return arc_support_shift_feedback(state,pose,phase,width,gain,0,0,0,0,0,0);
}
#endif

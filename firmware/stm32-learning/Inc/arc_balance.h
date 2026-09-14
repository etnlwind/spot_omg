#ifndef ARC_BALANCE_H
#define ARC_BALANCE_H
#include "arc_turn.h"
/* Cartesian attitude correction distinguishes load-bearing and airborne feet.
 * Support is scheduled; this is not measured contact or a torque controller. */
static inline bool arc_balance_targets(const GaitPolicyLegTarget nominal[4],float roll,float pitch,
        float phase,float duty,float gain,float swing_gain,GaitPolicyLegTarget out[4]) {
 if(!isfinite(roll)||!isfinite(pitch)||!isfinite(gain)||!isfinite(swing_gain))return false;
 for(int i=0;i<4;i++){
  float u=phase+((i==1||i==2)?.5f:0);u-=floorf(u);
  float support=1;
  if(u>duty-.08f && u<duty)support=1-gait_policy_smootherstep((u-duty+.08f)/.08f);
  else if(u>=duty)support=gait_policy_smootherstep(gait_policy_clampf((u-.92f)/.08f,0,1));
  float factor=gain*support-swing_gain*(1-support);
  float q[3]={nominal[i].j1_deg,nominal[i].j2_deg,nominal[i].j3_deg},point[3],goal[3];
  arc_foot(i,q,point,NULL);
  float r=gait_policy_clampf(roll*factor,-.07f,.07f),p=gait_policy_clampf(pitch*factor,-.07f,.07f);
  float cr=cosf(r),sr=sinf(r),cp=cosf(p),sp=sinf(p);
  float x=point[0]-arc_center[0],y=point[1]-arc_center[1],z=point[2]-arc_center[2];
  float rotated[3]={cp*x+sp*(sr*y+cr*z),cr*y-sr*z,-sp*x+cp*(sr*y+cr*z)};
  for(int k=0;k<3;k++)goal[k]=point[k]+gait_policy_clampf(rotated[k]+arc_center[k]-point[k],-.01f,.01f);
  if(!arc_ik(i,goal,q))return false;
  out[i]=(GaitPolicyLegTarget){q[0],q[1],q[2],nominal[i].stance};
 }
 return true;
}
#endif

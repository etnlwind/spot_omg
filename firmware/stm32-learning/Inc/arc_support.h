#ifndef ARC_SUPPORT_H
#define ARC_SUPPORT_H
#include "arc_transfer.h"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "fp-contract=off")
#endif

/* Explicit experimental profile: wave60_rank3, nominal 11.1V CAD plant.
 * Not a validated gait: per-cycle clearance/contact/tracking still fail.
 * Fixed feedforward only; independent IMU safety remains enabled. */
static inline bool arc_support_targets(float phase,float scale,float linear,float yaw,GaitPolicyLegTarget out[4]) {
    if(!arc_turn_targets_configured(phase,scale,linear,yaw,.022f,.5f,.04f,0,out))return false;
    float fraction=fabsf(yaw)/fmaxf(fabsf(linear)+fabsf(yaw),1.e-9f);
    float phi=2*GAIT_POLICY_PI*(phase+.04f/(1.44f-.24f*fraction));
    float c=cosf(phi),s=sinf(phi),activity=scale*fminf(1,fabsf(linear)+fabsf(yaw));
    float dx=activity*(.001755704036f-.000990532463f*(c*c-s*s)+.000780877233f*2*c*s);
    float dy=activity*(.001239548517f*c+.001311364329f*s);
    if(dx*dx+dy*dy>.0001f)return false;
    if(activity==0)return true;
    for(int i=0;i<4;i++){
        float q[3]={out[i].j1_deg,out[i].j2_deg,out[i].j3_deg},point[3];
        arc_foot(i,q,point,NULL);point[0]-=dx;point[1]-=dy;
        if(!arc_ik(i,point,q))return false;
        out[i].j1_deg=q[0];out[i].j2_deg=q[1];out[i].j3_deg=q[2];
    }
    return true;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

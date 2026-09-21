#ifndef FOOT_LIFT_H
#define FOOT_LIFT_H
#include "locomotion.h"
/* Session-wide per-foot Cartesian swing offsets. Zero is an exact no-op. */
static inline bool foot_lift_apply(const uint32_t mm[4],float phase,float duty,float scale,
        const float offsets[4],GaitPolicyLegTarget targets[4]) {
    float delta[4];bool any=false;
    for(int i=0;i<4;i++){delta[i]=mm[i]/1000.f;any|=mm[i]!=0;}
    if(!any || scale==0)return true;
    GaitPolicyLegTarget result[4];
    if(!arc_swing_shape_apply_unbounded(targets,phase,duty,scale,offsets,delta,result))return false;
    for(int i=0;i<4;i++)targets[i]=result[i];
    return true;
}
static inline bool foot_lift_profile(const uint32_t mm[4],int profile,float phase,float scale,
        float linear,float yaw,GaitPolicyLegTarget targets[4]) {
    float p[7];locomotion_params(profile,linear,p);
    float offsets[4]={0,.5f,.5f,0},duty=p[1];
    if(profile==0)duty=GAIT_POLICY_TROT4_DUTY;
    if(profile==1){offsets[2]=.75f;offsets[3]=.25f;}
    if(profile==locomotion_profile_id("arcturn") || profile==locomotion_profile_id("arcsupport")) {
        float fraction=fabsf(yaw)/fmaxf(fabsf(linear)+fabsf(yaw),1.e-9f);
        phase+=.04f/(1.44f-.24f*fraction);duty=.5f;
    }
    return foot_lift_apply(mm,phase,duty,scale,offsets,targets);
}
#endif

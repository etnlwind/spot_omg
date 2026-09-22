#ifndef FOOT_LIFT_H
#define FOOT_LIFT_H
#include "locomotion.h"
#include <string.h>
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("Os", "fp-contract=off")
#endif
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
/* Legacy/shared gait targets are motor angles. Front J1 is opposite to CAD
 * (STS3250-POSITION-CONTROL.md); native CAD targets use their own adapter.
 * Convert at this boundary so actual nominal X/Z is preserved, not the
 * fictitious foot obtained by treating front motor angles as CAD angles. */
static inline bool foot_width_apply_coordinates(const int32_t mm[4],float scale,bool cad,GaitPolicyLegTarget targets[4]) {
    if(!isfinite(scale)||scale<0||scale>1)return false;
    GaitPolicyLegTarget result[4];
    for(int i=0;i<4;i++) {
        result[i]=targets[i];
        if(!mm[i] || scale==0)continue;
        float q[3]={targets[i].j1_deg,targets[i].j2_deg,targets[i].j3_deg},goal[3];
        if(i<2 && !cad)q[0]=-q[0];
        arc_foot(i,q,goal,NULL);
        goal[1]+=(i%2==0?1.f:-1.f)*mm[i]*.001f*scale;
        if(!arc_swing_shape_refine(i,goal,q))return false;
        result[i].j1_deg=i<2 && !cad?-q[0]:q[0];result[i].j2_deg=q[1];result[i].j3_deg=q[2];
    }
    memcpy(targets,result,sizeof(result));return true;
}
static inline bool foot_width_apply(const int32_t mm[4],float scale,GaitPolicyLegTarget targets[4]) {
    return foot_width_apply_coordinates(mm,scale,false,targets);
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
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

#ifndef ARC_SWING_SHAPE_H
#define ARC_SWING_SHAPE_H
#include "arc_turn.h"

/* A C2 Cartesian swing-height increment. The base gait clock and stance are
 * unchanged. Per-leg deltas are explicit caller inputs, never auto-reduced.
 * Floating-point IK/mesh geometry and later servo quantization retain their
 * own numerical precision; the requested height envelope is C2. */
static inline float arc_swing_shape_envelope(float phase,float duty) {
    float p=phase-floorf(phase);
    if(p<duty)return 0;
    float u=(p-duty)/(1.f-duty),v=1.f-u;
    return 64.f*u*u*u*v*v*v;
}

static inline bool arc_swing_shape_refine(int leg,const float goal[3],float q[3]) {
    if(!arc_ik(leg,goal,q))return false;
    const float low[3]={-30,-45,0},high[3]={30,100,150};
    /* Existing IK accepts 0.1mm early. Refine only this optional adapter so
     * small smooth height increments do not acquire that large deadband. */
    for(int iteration=0;iteration<8;iteration++){
        float point[3],jac[3][3],error[3],norm=0,a[3][4]={{0}},dq[3];
        arc_foot(leg,q,point,jac);
        for(int k=0;k<3;k++){error[k]=goal[k]-point[k];norm+=error[k]*error[k];}
        if(norm<1.e-12f)return true;
        for(int j=0;j<3;j++){
            for(int k=0;k<3;k++)for(int r=0;r<3;r++)a[j][k]+=jac[r][j]*jac[r][k];
            a[j][j]+=1.e-8f;for(int r=0;r<3;r++)a[j][3]+=jac[r][j]*error[r];
        }
        if(!arc_solve3(a,dq))return false;
        for(int j=0;j<3;j++)q[j]=gait_policy_clampf(q[j]+gait_policy_clampf(dq[j]*180.f/GAIT_POLICY_PI,-4,4),low[j],high[j]);
    }
    float point[3],error=0;arc_foot(leg,q,point,NULL);
    for(int k=0;k<3;k++)error+=(point[k]-goal[k])*(point[k]-goal[k]);
    return error<1.e-10f;
}

static inline bool arc_swing_shape_apply(const GaitPolicyLegTarget nominal[4],
        float phase,float duty,float scale,const float offsets[4],const float delta_m[4],
        GaitPolicyLegTarget out[4]) {
    if(!nominal||!offsets||!delta_m||!out||!isfinite(phase)||!isfinite(duty)||duty<.5f||duty>.85f||
       !isfinite(scale)||scale<0||scale>1)return false;
    for(int i=0;i<4;i++){
        if(!isfinite(offsets[i])||offsets[i]<0||offsets[i]>=1||!isfinite(delta_m[i])||fabsf(delta_m[i])>.01f||
           !isfinite(nominal[i].j1_deg)||!isfinite(nominal[i].j2_deg)||!isfinite(nominal[i].j3_deg)||
           nominal[i].j1_deg< -30||nominal[i].j1_deg>30||nominal[i].j2_deg< -45||nominal[i].j2_deg>100||
           nominal[i].j3_deg<0||nominal[i].j3_deg>150)return false;
    }
    GaitPolicyLegTarget result[4];
    for(int i=0;i<4;i++){
        result[i]=nominal[i];
        float dz=scale*delta_m[i]*arc_swing_shape_envelope(phase+offsets[i],duty);
        if(fabsf(dz)<1.e-9f)continue;
        float q[3]={nominal[i].j1_deg,nominal[i].j2_deg,nominal[i].j3_deg},goal[3];
        arc_foot(i,q,goal,NULL);goal[2]+=dz;
        if(!arc_swing_shape_refine(i,goal,q))return false;
        result[i].j1_deg=q[0];result[i].j2_deg=q[1];result[i].j3_deg=q[2];
    }
    for(int i=0;i<4;i++)out[i]=result[i];
    return true;
}
#endif

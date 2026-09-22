#ifndef ATTITUDE_PD_H
#define ATTITUDE_PD_H
#include "body_stabilizer.h"
#include "body_stabilization_config.h"
#include "arc_turn.h"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("Os", "fp-contract=off")
#endif

/* Compatibility adapter: recover the EXISTING nominal CAD cushion trajectory,
 * add Cartesian offsets, then perform the final IK. The controller itself has
 * no joint-angle or servo dependencies. Canonical angles stay canonical. */
typedef struct {
    BodyStabilizerState body;
    bool ik_failed;
    float max_ik_error_m;
} AttitudePd;
static inline const char *attitude_pd_status_name(const AttitudePd *s) {
    return s->ik_failed?"ik-infeasible":body_stabilizer_status_name(s->body.diagnostics.status);
}

/* Refine the existing IK using its CAD Jacobian and unchanged joint bounds.
 * A 5mm residual needs a tighter solution than the gait's 0.5mm tolerance. */
static inline bool attitude_pd_refine(int leg,const float target[3],float q[3]) {
    if(!arc_ik(leg,target,q))return false;
    const float low[3]={-30,-45,0},high[3]={30,100,150};
    for(int it=0;it<8;it++) {
        float point[3],jac[3][3],a[3][4]={{0}},dq[3],error[3],distance=0;
        arc_foot(leg,q,point,jac);
        for(int k=0;k<3;k++){error[k]=target[k]-point[k];distance+=error[k]*error[k];}
        if(distance<1.e-12f)return true;
        for(int j=0;j<3;j++) {
            for(int k=0;k<3;k++)for(int r=0;r<3;r++)a[j][k]+=jac[r][j]*jac[r][k];
            a[j][j]+=1.e-9f;for(int r=0;r<3;r++)a[j][3]+=jac[r][j]*error[r];
        }
        if(!arc_solve3(a,dq))return false;
        for(int j=0;j<3;j++)q[j]=gait_policy_clampf(q[j]+gait_policy_clampf(dq[j]*180.f/GAIT_POLICY_PI,-4,4),low[j],high[j]);
    }
    return false;
}

static inline bool attitude_pd_apply_offsets(AttitudePd *s, const BodyStabilizerConfig *cfg,
        const BodyImuState *imu, uint32_t now, float dt, bool enabled,
        float phase, float period, float duty, bool moving,const float phase_offsets[4],
        const GaitPolicyLegTarget nominal[4], GaitPolicyLegTarget output[4]) {
    float feet[4][3], body_feet[4][3], offsets[4][3], weights[4] = {0};
    if (!s || !cfg || !nominal || !output) return false;
    for (int i=0;i<4;i++) {
        float q[3]={nominal[i].j1_deg,nominal[i].j2_deg,nominal[i].j3_deg};
        for(int j=0;j<3;j++) if(!isfinite(q[j])) return false;
        arc_foot(i,q,feet[i],NULL);
        for(int j=0;j<3;j++) body_feet[i][j]=feet[i][j]-arc_center[j];
    }
    if (moving && !body_stabilizer_stance_weights_offsets(phase,duty,period,cfg->stance_transition_s,phase_offsets,weights)) {
        s->body.diagnostics.status=BODY_STABILIZER_INVALID_GEOMETRY;return false;
    }
    AttitudePd next=*s;
    if(!body_stabilizer_update(&next.body,cfg,imu,now,dt,enabled && moving,
                              body_feet,weights,offsets)) {
        s->body.diagnostics=next.body.diagnostics;return false;
    }
    GaitPolicyLegTarget solved[4]; next.ik_failed=false; next.max_ik_error_m=0;
    for(int i=0;i<4;i++) {
        solved[i]=nominal[i];
        if(fabsf(offsets[i][2])<1.e-9f) continue; /* OFF is bit-identical. */
        float q[3]={nominal[i].j1_deg,nominal[i].j2_deg,nominal[i].j3_deg};
        /* Numerical margin stays inside the configured total Cartesian cap. */
        float dz=body_stabilizer_clip(offsets[i][2],fmaxf(0,cfg->max_foot_offset_m-5.e-6f));
        if(dz!=offsets[i][2])next.body.diagnostics.foot_clamp_mask|=(uint8_t)(1U<<i);
        float goal[3]={feet[i][0],feet[i][1],feet[i][2]+dz};
        if(!attitude_pd_refine(i,goal,q)) {s->ik_failed=true;return false;}
        float actual[3],error=0,displacement=0;arc_foot(i,q,actual,NULL);
        for(int j=0;j<3;j++) {
            error+=(actual[j]-goal[j])*(actual[j]-goal[j]);
            displacement+=(actual[j]-feet[i][j])*(actual[j]-feet[i][j]);
        }
        next.max_ik_error_m=fmaxf(next.max_ik_error_m,sqrtf(error));
        if(error>1.e-12f || displacement>cfg->max_foot_offset_m*cfg->max_foot_offset_m) {s->ik_failed=true;return false;}
        next.body.diagnostics.applied_dz[i]=actual[2]-feet[i][2];
        solved[i].j1_deg=q[0];solved[i].j2_deg=q[1];solved[i].j3_deg=q[2];
    }
    /* Atomic commit: failure never publishes a partially solved set of legs. */
    *s=next;
    for(int i=0;i<4;i++)output[i]=solved[i];
    return true;
}
static inline bool attitude_pd_apply(AttitudePd *s, const BodyStabilizerConfig *cfg,
        const BodyImuState *imu,uint32_t now,float dt,bool enabled,
        float phase,float period,float duty,bool moving,
        const GaitPolicyLegTarget nominal[4],GaitPolicyLegTarget output[4]) {
    const float offsets[4]={0,.5f,.5f,0};
    return attitude_pd_apply_offsets(s,cfg,imu,now,dt,enabled,phase,period,duty,moving,offsets,nominal,output);
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

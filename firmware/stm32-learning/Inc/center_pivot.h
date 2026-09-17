#ifndef CENTER_PIVOT_H
#define CENTER_PIVOT_H
#include "arc_swing_shape.h"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "fp-contract=off")
#endif
/* Port of the recorded inside_60_4 candidate. CAD cushion XY center and
 * lowest Z vertex, canonical degrees, no simulator pose/contact input.
 * Keep the simultaneous 12-iteration stopping criterion of SupportShift. */
static inline bool center_pivot_solve(const float goal[4][3],float q[4][3]) {
    const float low[3]={-30,-45,0},high[3]={30,100,150};
    for(int iteration=0;iteration<12;iteration++){
        float increment[4][3],maximum=0;
        for(int i=0;i<4;i++){
            float point[3],jac[3][3],error[3],a[3][4]={{0}},dq[3],norm=0;
            arc_foot(i,q[i],point,jac);
            for(int r=0;r<3;r++){error[r]=goal[i][r]-point[r];norm+=error[r]*error[r];}
            maximum=fmaxf(maximum,norm);
            for(int j=0;j<3;j++){
                for(int k=0;k<3;k++)for(int r=0;r<3;r++)a[j][k]+=jac[r][j]*jac[r][k];
                a[j][j]+=1.e-7f;
                for(int r=0;r<3;r++)a[j][3]+=jac[r][j]*error[r];
            }
            if(!arc_solve3(a,dq))return false;
            for(int j=0;j<3;j++)increment[i][j]=gait_policy_clampf(dq[j]*180.f/GAIT_POLICY_PI,-4,4);
        }
        if(maximum<1.e-8f)break;
        for(int i=0;i<4;i++)for(int j=0;j<3;j++)q[i][j]=gait_policy_clampf(q[i][j]+increment[i][j],low[j],high[j]);
    }
    for(int i=0;i<4;i++){
        float point[3],error=0;arc_foot(i,q[i],point,NULL);
        for(int j=0;j<3;j++)error+=(point[j]-goal[i][j])*(point[j]-goal[i][j]);
        if(!isfinite(error)||error>1.e-6f)return false;
    }
    return true;
}
static inline bool center_pivot_targets(const float p[7],float phase,float scale,float linear,float yaw,GaitPolicyLegTarget out[4]) {
    GaitPolicyLegTarget nominal[4],lifted[4];
    if(!locomotion_foot_targets(p,phase,scale,0,linear,yaw,nominal))return false;
    const float offsets[4]={0,.5f,.5f,0};
    float pivot=gait_policy_smootherstep(gait_policy_clampf(1-fabsf(linear)/.6f,0,1));
    float turn=pivot*gait_policy_smootherstep(gait_policy_clampf(fabsf(yaw)/.25f,0,1));
    float activity=fminf(1,fabsf(linear)+(1+pivot)*fabsf(yaw));
    for(int i=0;i<4;i++){
        float q=phase+offsets[i];q-=floorf(q);
        if(q<p[1])continue;
        float u=(q-p[1])/(1-p[1]),k=(1-p[1])/p[1];
        float x=p[2]*(-.5f+(1+k)*gait_policy_smootherstep(u)-k*u);
        float held=u<.4f?gait_policy_smootherstep(u/.4f):u>.7f?gait_policy_smootherstep((1-u)/.3f):1.f;
        float arch=64*u*u*u*(1-u)*(1-u)*(1-u);
        float envelope=arch+(held-arch)*gait_policy_smootherstep(gait_policy_clampf(linear/.5f,0,1));
        float z=p[4]-scale*activity*p[3]*envelope;
        float lateral=-scale*yaw*x*(i<2?1.6f:-1.6f)*pivot;
        x=p[5]+scale*gait_policy_clampf(linear+(i%2==0?yaw:-yaw),-1,1)*x;
        float c=(x*x+z*z-.141f*.141f-.150f*.150f)/(2*.141f*.150f);
        if(c< -1||c>1)return false;
        float knee=acosf(c),hip=atan2f(-x,z)+atan2f(.150f*sinf(knee),.141f+.150f*cosf(knee));
        nominal[i].j1_deg=p[6]+atan2f(lateral*(i%2==0?1:-1),z)*180.f/GAIT_POLICY_PI;
        nominal[i].j2_deg=hip*180.f/GAIT_POLICY_PI;nominal[i].j3_deg=knee*180.f/GAIT_POLICY_PI;
    }
    float extra[4]={0};extra[yaw<0?0:1]=.004f;
    if(!arc_swing_shape_apply(nominal,phase,p[1],scale*turn,offsets,extra,lifted))return false;
    if(turn<=0){for(int i=0;i<4;i++)out[i]=lifted[i];return true;}
    float q[4][3],goal[4][3];
    for(int i=0;i<4;i++){
        q[i][0]=lifted[i].j1_deg;q[i][1]=lifted[i].j2_deg;q[i][2]=lifted[i].j3_deg;
        arc_foot(i,q[i],goal[i],NULL);
        float phase_i=phase+offsets[i];phase_i-=floorf(phase_i);
        float x;
        if(phase_i<p[1])x=.5f-phase_i/p[1];
        else {float u=(phase_i-p[1])/(1-p[1]),k=(1-p[1])/p[1];x=-.5f+(1+k)*gait_policy_smootherstep(u)-k*u;}
        float angle=-scale*yaw*.24f*x;
        goal[i][0]+=(1-cosf(angle))*.06f;goal[i][1]-=sinf(angle)*.06f;
    }
    if(!center_pivot_solve(goal,q))return false;
    for(int i=0;i<4;i++){
        out[i]=lifted[i];
        out[i].j1_deg+=turn*(q[i][0]-out[i].j1_deg);
        out[i].j2_deg+=turn*(q[i][1]-out[i].j2_deg);
        out[i].j3_deg+=turn*(q[i][2]-out[i].j3_deg);
    }
    return true;
}
/* V2 retains the rear targets and adds only forward front swing clearance.
 * S is the zero-amplitude entry posture. Full-amplitude rear motion is exact.
 * CAD FK/IK preserves front X/Y; no motor limits or calibrations change. */
#include "s_native_data.h"
static inline bool attitude_v2_targets(const float p[7],float phase,float scale,float linear,float yaw,GaitPolicyLegTarget out[4]) {
    GaitPolicyLegTarget base[4],result[4];
    if(!center_pivot_targets(p,phase,scale,linear,yaw,base))return false;
    float entry=gait_policy_smootherstep(scale);
    for(int i=0;scale<1 && i<4;i++) {
        base[i].j1_deg=sn_standing[i][0]+entry*(base[i].j1_deg-sn_standing[i][0]);
        base[i].j2_deg=sn_standing[i][1]+entry*(base[i].j2_deg-sn_standing[i][1]);
        base[i].j3_deg=sn_standing[i][2]+entry*(base[i].j3_deg-sn_standing[i][2]);
    }
    const float offsets[4]={0,.5f,.5f,0},extra[4]={.004f,.004f,0,0};
    float forward=gait_policy_smootherstep(gait_policy_clampf(linear/.5f,0,1));
    if(!arc_swing_shape_apply(base,phase,p[1],scale*forward,offsets,extra,result))return false;
    for(int i=0;i<4;i++)out[i]=result[i];
    return true;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

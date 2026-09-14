#ifndef ARC_LIFT_FIRST_H
#define ARC_LIFT_FIRST_H
#include "arc_turn.h"
/* Experimental Cartesian swing scheduling. Stance speed, period and sweep
 * are retained. Decelerate the inherited stance velocity before transferring
 * the foot, then match it again at landing. C2 at all segment boundaries.
 * This is planned clearance, not a measured-contact readiness detector. */
static inline float arc_lift_first_x(float u,float duty,float fraction) {
    float k=(1-duty)/duty;
    if(u>1-fraction)return -arc_lift_first_x(1-u,duty,fraction);
    if(u<fraction){
        float t=u/fraction;
        return -.5f-k*fraction*(t-t*t*t+.5f*t*t*t*t);
    }
    float extent=.5f+.5f*k*fraction;
    return extent*(2*gait_policy_smootherstep((u-fraction)/(1-2*fraction))-1);
}
static inline bool arc_lift_first_apply(GaitPolicyLegTarget pose[4],float phase,
        float scale,float linear,float yaw,float duty,float sweep,float fraction) {
    if(!isfinite(phase)||!isfinite(scale)||scale<0||scale>1||
       !isfinite(linear)||fabsf(linear)>1||!isfinite(yaw)||fabsf(yaw)>1||
       !isfinite(duty)||duty<.5f||duty>.65f||!isfinite(sweep)||sweep<.05f||sweep>.4f||
       !isfinite(fraction)||fraction<.05f||fraction>.3f)return false;
    const float offsets[4]={0,.5f,.5f,0};
    if(!pose)return false;
    for(int i=0;i<4;i++){
        if(!isfinite(pose[i].j1_deg)||!isfinite(pose[i].j2_deg)||!isfinite(pose[i].j3_deg)||
           pose[i].j1_deg< -30||pose[i].j1_deg>30||pose[i].j2_deg< -45||pose[i].j2_deg>100||
           pose[i].j3_deg<0||pose[i].j3_deg>150)return false;
    }
    GaitPolicyLegTarget result[4];
    for(int i=0;i<4;i++){
        result[i]=pose[i];float p=phase+offsets[i];p-=floorf(p);
        if(p<duty)continue;
        float u=(p-duty)/(1-duty),k=(1-duty)/duty;
        float oldx=-.5f+(1+k)*gait_policy_smootherstep(u)-k*u;
        float x=arc_lift_first_x(u,duty,fraction);
        float a=-scale*yaw*sweep*oldx,b=-scale*yaw*sweep*x;
        float dx=arc_reference[i][0]-arc_center[0],dy=arc_reference[i][1]-arc_center[1];
        float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg},point[3];
        arc_foot(i,q,point,NULL);
        point[0]+=(cosf(b)-cosf(a))*dx-(sinf(b)-sinf(a))*dy+scale*linear*.08f*(x-oldx);
        point[1]+=(sinf(b)-sinf(a))*dx+(cosf(b)-cosf(a))*dy;
        if(!arc_ik(i,point,q))return false;
        result[i].j1_deg=q[0];result[i].j2_deg=q[1];result[i].j3_deg=q[2];
    }
    for(int i=0;i<4;i++)pose[i]=result[i];
    return true;
}
#endif

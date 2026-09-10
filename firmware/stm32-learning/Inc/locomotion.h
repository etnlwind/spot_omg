#ifndef LOCOMOTION_H
#define LOCOMOTION_H
#include "gait_policy.h"
#include "locomotion_profiles.h"
#include <string.h>

static inline int locomotion_profile_id(const char *name) {
    if (!name) return -1;
    for(int i=0;i<LOCOMOTION_PROFILE_COUNT;i++) if(strcmp(name,locomotion_names[i])==0) return i;
    return -1;
}
static inline float locomotion_linear(int profile,float linear) {
    return (profile==3 || profile==4 || profile==5 || profile==6 || profile==7 || profile==8 || profile==9 || profile==10 || profile==11) ? fmaxf(-.6f,linear) : linear;
}
static inline void locomotion_params(int profile,float linear,float p[7]) {
    float w=gait_policy_smootherstep(gait_policy_clampf(linear/.5f,0,1));
    for(int j=0;j<7;j++) p[j]=locomotion_parameters[profile][1][j]+w*(locomotion_parameters[profile][0][j]-locomotion_parameters[profile][1][j]);
}
/* family: 0 trot, 1 crawl/amble, 2 pace, 3 bound. */
static inline bool locomotion_foot_targets(const float p[7],float phase,float scale,
        int family,float linear,float yaw,GaitPolicyLegTarget out[4]) {
    static const float phases[4][4]={{0,.5f,.5f,0},{0,.5f,.75f,.25f},{0,.5f,0,.5f},{0,0,.5f,.5f}};
    if(family<0 || family>3 || !isfinite(phase) || !isfinite(scale) || !isfinite(linear) || !isfinite(yaw) || scale<0 || scale>1 || fabsf(linear)>1 || fabsf(yaw)>1) return false;
    for(int j=0;j<7;j++) if(!isfinite(p[j])) return false;
    if(p[0]<=0 || p[1]<.45f || p[1]>.9f) return false;
    float pivot=gait_policy_smootherstep(gait_policy_clampf(1-fabsf(linear)/.6f,0,1));
    for(int i=0;i<4;i++) {
        float q=phase+phases[family][i];q-=floorf(q);
        float command=gait_policy_clampf(linear+(i%2==0?yaw:-yaw),-1,1);
        float x,z=p[4];
        if(q<p[1]) x=p[2]*(.5f-q/p[1]);
        else {
            float u=(q-p[1])/(1-p[1]),k=(1-p[1])/p[1];
            x=p[2]*(-.5f+(1+k)*gait_policy_smootherstep(u)-k*u);
            z-=p[3]*64*u*u*u*(1-u)*(1-u)*(1-u);
        }
        float lateral=-scale*yaw*x*(i<2?1.6f:-1.6f)*pivot;
        float activity=fminf(1,fabsf(linear)+(1+pivot)*fabsf(yaw));
        x=p[5]+scale*command*x;z=p[4]+scale*activity*(z-p[4]);
        float c=(x*x+z*z-.141f*.141f-.150f*.150f)/(2*.141f*.150f);
        if(c < -1 || c > 1) return false;
        float knee=acosf(c),hip=atan2f(-x,z)+atan2f(.150f*sinf(knee),.141f+.150f*cosf(knee));
        out[i]=(GaitPolicyLegTarget){p[6]+atan2f(lateral*(i%2==0?1:-1),z)*57.295779513f,hip*57.295779513f,knee*57.295779513f,q<p[1]};
        if(out[i].j1_deg < -30 || out[i].j1_deg >30 || out[i].j2_deg < -45 || out[i].j2_deg>100 || out[i].j3_deg<0 || out[i].j3_deg>150) return false;
    }
    return true;
}
static inline float locomotion_turn_assist(int profile,float linear,float yaw) {
    if(profile<0 || profile>=LOCOMOTION_PROFILE_COUNT || !isfinite(linear) || !isfinite(yaw))return 0;
    return locomotion_turn_lift_m[profile]>0?
        gait_policy_smootherstep(gait_policy_clampf((.5f-linear)/.3f,0,1))*
        gait_policy_smootherstep(gait_policy_clampf(1+linear/.2f,0,1))*
        gait_policy_smootherstep(gait_policy_clampf(fabsf(yaw)/.25f,0,1)):0;
}
static inline bool locomotion_targets_assisted(int profile,float phase,float scale,float linear,float yaw,float assist,GaitPolicyLegTarget out[4]) {
    if(profile<0 || profile>=LOCOMOTION_PROFILE_COUNT || !isfinite(assist) || assist<0 || assist>1) return false;
    if(profile==0) return gait_policy_drive_walk_targets(phase,scale,linear,yaw,out);
    float p[7];locomotion_params(profile,linear,p);
    /* Recenter the stance as the legs straighten: lift alone left the
     * front feet loaded against the floor in the estimated physical plant. */
    if(locomotion_turn_posture_m[profile][0]>0) {
        float w=assist;
        p[4]+=(locomotion_turn_posture_m[profile][0]-p[4])*w;
        p[5]+=(locomotion_turn_posture_m[profile][1]-p[5])*w;
    }
    /* Small turn commands must not shrink swing lift into ground clearance.
     * Ramp the lift floor continuously from zero yaw; IK coordinates J2/J3.
     * Startup/stop amplitude still scales the entire trajectory below. */
    float pivot=gait_policy_smootherstep(gait_policy_clampf(1-fabsf(linear)/.6f,0,1));
    float activity=fminf(1,fabsf(linear)+(1+pivot)*fabsf(yaw));
    float turn=assist;
    if(activity>1.e-6f)
        p[3]=fmaxf(p[3],locomotion_turn_lift_m[profile]*turn/activity);
    if(!locomotion_foot_targets(p,phase,scale,profile==1?1:0,linear,yaw,out))return false;
    const float *lead=locomotion_joint_lead_s[profile];
    if(lead[1]>0 || lead[2]>0) {
        /* Bounded velocity feed-forward for the existing position servos.
         * Coordinate J2 propulsion with J3 flexion; never increase motor limits. */
        GaitPolicyLegTarget a[4],b[4];
        float period=p[0]*(1.35f-.35f*fminf(1,fabsf(linear)+fabsf(yaw)));
        if(!locomotion_foot_targets(p,phase-.005f+1.f,scale,0,linear,yaw,a) ||
           !locomotion_foot_targets(p,phase+.005f,scale,0,linear,yaw,b))return false;
        for(int i=0;i<4;i++) {
            out[i].j2_deg+=gait_policy_clampf(lead[1]*(b[i].j2_deg-a[i].j2_deg)/(.01f*period),-6.f,6.f);
            out[i].j3_deg+=gait_policy_clampf(lead[2]*(b[i].j3_deg-a[i].j3_deg)/(.01f*period),-6.f,6.f);
            if(out[i].j2_deg < -45 || out[i].j2_deg >100 || out[i].j3_deg<0 || out[i].j3_deg>150)return false;
        }
    }
    return true;
}
static inline bool locomotion_targets(int profile,float phase,float scale,float linear,float yaw,GaitPolicyLegTarget out[4]) {
    return locomotion_targets_assisted(profile,phase,scale,linear,yaw,locomotion_turn_assist(profile,linear,yaw),out);
}
static inline float locomotion_period(int profile,float linear,float yaw) {
    if(profile==0) return gait_policy_drive_period_ms(lroundf(linear*1000),lroundf(yaw*1000))*.001f;
    float p[7];locomotion_params(profile,linear,p);
    return p[0]*(1.35f-.35f*fminf(1,fabsf(linear)+fabsf(yaw)));
}
#endif

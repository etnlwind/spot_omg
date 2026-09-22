#ifndef NAVIGATION_CONTROL_H
#define NAVIGATION_CONTROL_H
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("Os", "fp-contract=off")
#endif
/* V7/V8: joystick axes, not a body velocity command. Old profiles retain their
 * original interpretation. All targets here are CAD angles; the physical
 * front J1 reversal belongs only at the servo encoding boundary. */
static const float navigation_offsets[4]={.8f,.3f,.05f,.55f};
static const float navigation_left_offsets[4]={.3f,.8f,.55f,.05f};
/* V8: 0.6 s paired swing, 0.6 s four-foot transfer, alternating pairs.
 * Start with all four feet down while saved width settings blend in. */
#define NAVIGATION_PAIR_DUTY .75f
#define NAVIGATION_PAIR_PERIOD 2.4f
#define NAVIGATION_IPSILATERAL_PERIOD 4.8f
static const float navigation_pair_offsets[4]={NAVIGATION_PAIR_DUTY,NAVIGATION_PAIR_DUTY-.5f,NAVIGATION_PAIR_DUTY-.5f,NAVIGATION_PAIR_DUTY};
static const float navigation_ipsilateral_offsets[4]={NAVIGATION_PAIR_DUTY,NAVIGATION_PAIR_DUTY-.5f,NAVIGATION_PAIR_DUTY,NAVIGATION_PAIR_DUTY-.5f};
static const float navigation_instep_offsets[4]={.8f,.3f,.8f,.3f};
/* Positive Y is the robot's left. Swinging the left pair requires right support.
 * Transfer changes only during planned four-foot support. This feedforward
 * translation is NOT evidence that contact or COM actually follows the plan. */
static inline float navigation_ipsilateral_shift(float phase) {
    float p=gait_policy_wrap_phase(phase);
    if(p<.25f)return -1;
    if(p<.5f)return -1+2*gait_policy_smootherstep((p-.25f)*4);
    if(p<.75f)return 1;
    return 1-2*gait_policy_smootherstep((p-.75f)*4);
}
static inline bool navigation_ipsilateral_transfer(float phase,float scale,float distance,GaitPolicyLegTarget out[4]) {
    if(!isfinite(distance) || distance<0 || distance>.12f)return false;
    float shift=distance*scale*navigation_ipsilateral_shift(phase);
    for(int i=0;i<4;i++) {
        float q[3]={out[i].j1_deg,out[i].j2_deg,out[i].j3_deg},goal[3];
        arc_foot(i,q,goal,NULL);goal[1]-=shift;
        if(!arc_swing_shape_refine(i,goal,q))return false;
        out[i].j1_deg=q[0];out[i].j2_deg=q[1];out[i].j3_deg=q[2];
    }
    return true;
}
static inline float navigation_support_duty(const DriveControl *s) {
    if(s->left_instep && s->side_mode==1)return .8f;
    return s->paired_side && s->side_mode==1?NAVIGATION_PAIR_DUTY:.8f;
}
static inline float navigation_start_scale(const DriveControl *s) {
    return s->paired_side && s->side_mode==1?gait_policy_smootherstep(fminf(1,s->elapsed/1.5f)):
        locomotion_start_scale(locomotion_profile_id("attitudepd_v6"),s->elapsed,s->linear);
}
static inline const float *navigation_support_offsets(const DriveControl *s) {
    if(s->left_instep && s->side_mode==1)return navigation_instep_offsets;
    if(s->ipsilateral_side && s->side_mode==1)return navigation_ipsilateral_offsets;
    if(s->paired_side && s->side_mode==1)return navigation_pair_offsets;
    return s->side_mode==1 && s->lateral<0?navigation_left_offsets:navigation_offsets;
}
static inline float navigation_pair_lift_shape(float u) {
    return gait_policy_smootherstep(fminf(1,u/.25f))*gait_policy_smootherstep(fminf(1,(1-u)/.25f));
}
#include "lateral_instep.h"
static inline bool navigation_pair_extra_lift_offsets(const uint32_t mm[4],float phase,float scale,const float offsets[4],GaitPolicyLegTarget out[4]) {
    for(int i=0;i<4;i++) {
        float p=gait_policy_wrap_phase(phase+offsets[i]);
        if(!mm[i] || p<NAVIGATION_PAIR_DUTY || scale==0)continue;
        float u=(p-NAVIGATION_PAIR_DUTY)/(1-NAVIGATION_PAIR_DUTY);
        float q[3]={out[i].j1_deg,out[i].j2_deg,out[i].j3_deg},goal[3];
        arc_foot(i,q,goal,NULL);goal[2]+=mm[i]*.001f*scale*navigation_pair_lift_shape(u);
        if(!arc_swing_shape_refine(i,goal,q))return false;
        out[i].j1_deg=q[0];out[i].j2_deg=q[1];out[i].j3_deg=q[2];
    }
    return true;
}
static inline bool navigation_pair_extra_lift(const uint32_t mm[4],float phase,float scale,GaitPolicyLegTarget out[4]) {
    return navigation_pair_extra_lift_offsets(mm,phase,scale,navigation_pair_offsets,out);
}
static inline bool navigation_side_targets(float phase,float scale,float lateral,
        float correction,bool paired,bool ipsilateral,GaitPolicyLegTarget out[4]) {
    if(!locomotion_stand_targets(locomotion_profile_id("attitudepd_v7"),out))return false;
    for(int i=0;i<4;i++) {
        float q[3]={out[i].j1_deg,out[i].j2_deg,out[i].j3_deg},goal[3];
        arc_foot(i,q,goal,NULL);
        float duty=paired?NAVIGATION_PAIR_DUTY:.8f;
        float p=gait_policy_wrap_phase(phase+(ipsilateral?navigation_ipsilateral_offsets[i]:paired?navigation_pair_offsets[i]:navigation_offsets[lateral<0?(i^1):i]));
        bool stance=p<duty;
        float u=paired?(stance?p/duty:(p-duty)/(1-duty)):(stance?p/.8f:(p-.8f)/.2f);
        /* Every planted foot must have the SAME Cartesian velocity. A
         * separate eased stance wave per leg makes the support feet fight. */
        float k=(1-duty)/duty;
        float recovery=gait_policy_clampf((u-.2f)/.6f,0,1);
        float wave=paired?(stance?.5f-u:-.5f+(1+k)*gait_policy_smootherstep(recovery)-k*u):
            (stance?.5f-u:-.5f+1.25f*gait_policy_smootherstep(u)-.25f*u);
        float angle=.24f*scale*correction*wave;
        float x=goal[0]-arc_center[0],y=goal[1]-arc_center[1];
        goal[0]+=angle*y;
        goal[1]-=scale*lateral*.024f*wave+angle*x;
        if(!stance)goal[2]+=(paired?.012f:.024f)*scale*fminf(1,(fabsf(lateral)+fabsf(correction))/.15f)*
            (paired?navigation_pair_lift_shape(u):gait_policy_smootherstep(fminf(u,1-u)*2));
        if(!arc_swing_shape_refine(i,goal,q))return false;
        out[i]=(GaitPolicyLegTarget){q[0],q[1],q[2],stance};
    }
    return true;
}
/* Heading feedback changes signed fore/aft stride only. It cannot change
 * foot Y/Z, the gait's period, or lift. The 20% bound prevents reversing a
 * leg at small input; reverse naturally swaps the long/short stride side. */
static inline bool navigation_stride_correct(float linear,float correction,
        GaitPolicyLegTarget out[4]) {
    if(fabsf(linear)<.15f || fabsf(correction)<1.e-7f)return true;
    GaitPolicyLegTarget center[4];
    if(!locomotion_targets(locomotion_profile_id("attitudepd_v6"),.26f,1,linear,0,center))return false;
    GaitPolicyLegTarget other[4];
    if(!locomotion_targets(locomotion_profile_id("attitudepd_v6"),.76f,1,linear,0,other))return false;
    float ratio=gait_policy_clampf(correction/linear,-.2f,.2f);
    for(int i=0;i<4;i++) {
        float q[3]={out[i].j1_deg,out[i].j2_deg,out[i].j3_deg},goal[3],a[3],b[3];
        float qa[3]={i<2?-center[i].j1_deg:center[i].j1_deg,center[i].j2_deg,center[i].j3_deg};
        float qb[3]={i<2?-other[i].j1_deg:other[i].j1_deg,other[i].j2_deg,other[i].j3_deg};
        arc_foot(i,q,goal,NULL);arc_foot(i,qa,a,NULL);arc_foot(i,qb,b,NULL);
        goal[0]+=(goal[0]-(a[0]+b[0])*.5f)*ratio*(i%2==0?1:-1);
        if(!arc_swing_shape_refine(i,goal,q))return false;
        out[i].j1_deg=q[0];out[i].j2_deg=q[1];out[i].j3_deg=q[2];
    }
    return true;
}
static inline bool navigation_step(DriveControl *s,float linear,float yaw,float heading,
        bool valid,bool enabled,bool stopping,float dt,float rate,GaitPolicyLegTarget out[4]) {
    /* 0 longitudinal/arc, 1 lateral, 2 pivot. Crawl timing serves both
     * planar modes, but only lateral movement captures a heading. */
    int desired_side=linear==0 && yaw!=0?1:linear<0 && yaw!=0?2:0;
    bool requested_left=yaw<0;
    if(stopping){linear=0;yaw=0;desired_side=s->side_mode;}
    bool changing=(linear!=0 || yaw!=0) && desired_side!=s->side_mode;
    if(changing){linear=0;yaw=0;}
    float lateral=s->side_mode==1?yaw:0;
    if(s->side_mode==1)yaw=0;
    else if(s->side_mode==2)linear=0;
    yaw=gait_policy_clampf(yaw,-.5f,.5f);
    float progress=dt*rate;
    /* Full-scale normal stop takes at most 2 s before the existing 1 s
     * Stand blend. Heartbeats/IMU still run at 50 Hz; emergency abort is
     * handled by the outer robot loop and never waits for this ramp. */
    float slew_dt=progress*((stopping || changing)?.25f:1.f);
    s->linear=drive_timed_slew(lroundf(s->linear*1000),lroundf(linear*1000),slew_dt)*.001f;
    s->yaw=drive_timed_slew(lroundf(s->yaw*1000),lroundf(yaw*1000),slew_dt)*.001f;
    s->lateral=drive_timed_slew(lroundf(s->lateral*1000),lroundf(lateral*1000),slew_dt)*.001f;
    int direction=linear>0?1:linear<0?-1:0;
    if(direction!=s->heading_direction){s->heading=(HeadingControl){0};s->heading_direction=direction;}
    float activity=s->side_mode==1?fabsf(lateral):fabsf(linear);
    float correction=heading_update(&s->heading,heading,valid,
        enabled && !stopping && !changing,activity,yaw,s->yaw,dt);
    /* Slow cadence, not command magnitude: reverse keeps its lift/template.
     * Stop and mode-change deceleration continue at the normal control rate. */
    s->navigation_period=s->side_mode?(s->side_mode==1?(s->left_instep?2.5f:s->ipsilateral_side?NAVIGATION_IPSILATERAL_PERIOD:s->paired_side?NAVIGATION_PAIR_PERIOD:4.4f):3.4f):
        locomotion_period(locomotion_profile_id("attitudepd_v6"),s->linear,s->yaw)*
        (s->linear<0 && s->yaw==0?2.f:1.f);
    s->elapsed+=progress;
    float scale=navigation_start_scale(s);
    bool ok;
    if(s->left_instep)ok=locomotion_stand_targets(locomotion_profile_id("attitudepd_v7"),out);
    else if(s->side_mode)ok=navigation_side_targets(s->phase,scale,s->lateral,s->yaw+correction,s->paired_side && s->side_mode==1,s->ipsilateral_side && s->side_mode==1,out);
    else {
        ok=locomotion_targets(locomotion_profile_id("attitudepd_v6"),s->phase,scale,s->linear,s->yaw,out);
        for(int i=0;i<2;i++)out[i].j1_deg=-out[i].j1_deg;
        if(ok && yaw==0 && fabsf(s->yaw)<.001f)ok=navigation_stride_correct(s->linear,correction,out);
    }
    if(!(s->paired_side && s->side_mode==1 && s->elapsed<1.5f))
        s->phase=s->left_instep?fminf(.45f,s->phase+progress/s->navigation_period):gait_policy_wrap_phase(s->phase+progress/s->navigation_period);
    if(changing && fabsf(s->linear)+fabsf(s->yaw)+fabsf(s->lateral)<.001f) {
        s->side_mode=desired_side;s->phase=s->ipsilateral_side && desired_side==1?(requested_left?.75f:.25f):s->paired_side && desired_side==1?1.5f-NAVIGATION_PAIR_DUTY:0;
        s->elapsed=0;s->heading=(HeadingControl){0};
        if(s->left_instep)s->phase=0;
        ok=locomotion_stand_targets(locomotion_profile_id("attitudepd_v7"),out);
    }
    return ok;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

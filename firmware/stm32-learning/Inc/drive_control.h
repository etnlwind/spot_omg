#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H
#include "locomotion.h"
#include "heading_control.h"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "fp-contract=off")
#endif
static inline int16_t drive_timed_slew(int16_t current,int16_t target,float dt) {
 int limit=(int)lroundf(2000*dt),difference=(int)target-current;
 return (int16_t)(current+(difference>limit?limit:(difference < -limit?-limit:difference)));
}
typedef struct {float phase,linear,yaw,elapsed;HeadingControl heading;float turn_assist;float arc_support_preload[12];} DriveControl;
static inline bool drive_control_step_timed(DriveControl *s,int profile,float linear,float yaw,
        float heading,bool valid,bool heading_enabled,bool stopping,float dt,float phase_rate,GaitPolicyLegTarget out[4]) {
    if(profile<0 || profile>=LOCOMOTION_PROFILE_COUNT || !isfinite(linear) || !isfinite(yaw) || fabsf(linear)>1 || fabsf(yaw)>1)return false;
    if(!isfinite(dt)||dt<=0||dt>.06f||!isfinite(phase_rate)||phase_rate<0||phase_rate>1)return false;
    float progress_dt=dt*phase_rate;
    s->elapsed+=progress_dt;
    linear=locomotion_linear(profile,linear);
    if(profile!=locomotion_profile_id("arcturn") && profile!=locomotion_profile_id("arcsupport"))yaw=gait_policy_drive_yaw_limit(lroundf(yaw*1000))*.001f;
    s->linear=drive_timed_slew(lroundf(s->linear*1000),lroundf(linear*1000),progress_dt)*.001f;
    float correction=heading_update(&s->heading,heading,valid,heading_enabled && !stopping,linear,yaw,s->yaw,dt);
    s->yaw=drive_timed_slew(lroundf(s->yaw*1000),lroundf((yaw+correction)*1000),progress_dt)*.001f;
    float wanted=locomotion_turn_assist(profile,linear,yaw);
    /* One second for a full posture change. Selecting from the requested
     * command avoids briefly entering the turn stance while slewing through
     * low speed on the way to a fast arc. Stop/abort handling remains outside. */
    s->turn_assist+=gait_policy_clampf(wanted-s->turn_assist,-progress_dt,progress_dt);
    bool ok=locomotion_targets_assisted(profile,s->phase,gait_policy_smootherstep(fminf(1,s->elapsed)),s->linear,s->yaw,s->turn_assist,out);
    if(ok && profile==locomotion_profile_id("arcsupport")){
        float activity=fminf(1,fabsf(s->linear)+fabsf(s->yaw));
        float fraction=fabsf(s->yaw)/fmaxf(fabsf(s->linear)+fabsf(s->yaw),1.e-9f);
        float target_phase=s->phase+.04f/(1.44f-.24f*fraction);
        ok=arc_transfer_apply(s->arc_support_preload,out,target_phase,
            locomotion_period(profile,s->linear,s->yaw),.5f,.12f,0,.3f*activity,35.f,dt);
    }
    s->phase=fmodf(s->phase+dt*phase_rate/locomotion_period(profile,s->linear,s->yaw),1.f);
    return ok;
}
/* Compatibility/reference path: deterministic 50Hz, no supervisor. */
static inline bool drive_control_step(DriveControl *s,int profile,float linear,float yaw,
        float heading,bool valid,bool heading_enabled,bool stopping,GaitPolicyLegTarget out[4]) {
    return drive_control_step_timed(s,profile,linear,yaw,heading,valid,heading_enabled,stopping,.02f,1,out);
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

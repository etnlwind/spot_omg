#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H
#include "locomotion.h"
#include "heading_control.h"
typedef struct {float phase,linear,yaw,elapsed;HeadingControl heading;float turn_assist;} DriveControl;
static inline bool drive_control_step(DriveControl *s,int profile,float linear,float yaw,
        float heading,bool valid,bool heading_enabled,bool stopping,GaitPolicyLegTarget out[4]) {
    if(profile<0 || profile>=LOCOMOTION_PROFILE_COUNT || !isfinite(linear) || !isfinite(yaw) || fabsf(linear)>1 || fabsf(yaw)>1)return false;
    s->elapsed+=.02f;
    linear=locomotion_linear(profile,linear);
    yaw=gait_policy_drive_yaw_limit(lroundf(yaw*1000))*.001f;
    s->linear=gait_policy_drive_slew(lroundf(s->linear*1000),lroundf(linear*1000))*.001f;
    float correction=heading_update(&s->heading,heading,valid,heading_enabled && !stopping,linear,yaw,s->yaw,.02f);
    s->yaw=gait_policy_drive_slew(lroundf(s->yaw*1000),lroundf((yaw+correction)*1000))*.001f;
    float wanted=locomotion_turn_assist(profile,linear,yaw);
    /* One second for a full posture change. Selecting from the requested
     * command avoids briefly entering the turn stance while slewing through
     * low speed on the way to a fast arc. Stop/abort handling remains outside. */
    s->turn_assist+=gait_policy_clampf(wanted-s->turn_assist,-.02f,.02f);
    bool ok=locomotion_targets_assisted(profile,s->phase,gait_policy_smootherstep(fminf(1,s->elapsed)),s->linear,s->yaw,s->turn_assist,out);
    s->phase=fmodf(s->phase+.02f/locomotion_period(profile,s->linear,s->yaw),1.f);
    return ok;
}
#endif

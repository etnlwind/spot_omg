#ifndef LATERAL_INSTEP_H
#define LATERAL_INSTEP_H
/* V10 revised LEFT first-step experiment. Positive CAD Y is left.
 * FL/RL lift using J2/J3 and translate using J1. FR/RR stay planted;
 * their J1 changes as the body translates left. IK also preserves stance
 * X/Z rather than dragging a planted foot. After left touchdown the body
 * target stops advancing: FR/RR lift and recover to Stand. Hold the final
 * pose; never wrap into a second step. Contact timing requires validation. */
static inline float lateral_instep_progress(float phase,float begin,float end) {
    return gait_policy_smootherstep(gait_policy_clampf((phase-begin)/(end-begin),0,1));
}
/* cfg: step metres, base lift metres, swing phase, push fraction,
 * catch phase duration, hip delay as a fraction of swing. Offline search
 * supplies these explicitly; defaults are not certified for hardware. */
static inline bool lateral_instep_configured(float phase,float scale,float input,
        const uint32_t lift_mm[4],const float cfg[6],GaitPolicyLegTarget target[4]) {
    float p=gait_policy_clampf(phase,0,.45f),step=cfg[0]*fabsf(input);
    float duration=cfg[2],recovery=duration+.01f;
    float push=step*cfg[3];
    float body=push*lateral_instep_progress(p,0,duration)
        +(step-push)*lateral_instep_progress(p,recovery+.35f*duration,recovery+.35f*duration+cfg[4]);
    for(int i=0;i<4;i++) {
        bool left=i%2==0;
        float u=gait_policy_clampf((p-(left?0:recovery))/duration,0,1);
        float travel=step*lateral_instep_progress(u,.35f,.8f);
        float q[3]={target[i].j1_deg,target[i].j2_deg,target[i].j3_deg},goal[3];
        arc_foot(i,q,goal,NULL);
        float base_y=goal[1];
        {
            float folded[3]={q[0],q[1],q[2]},raised[3]={goal[0],goal[1],goal[2]+scale*(cfg[1]+.001f*lift_mm[i])};
            if(!arc_swing_shape_refine(i,raised,folded))return false;
            /* Start knee folding before hip motion. Keep the resulting small
             * sagittal arc rather than assuming instant equal motor response. */
            float knee=lateral_instep_progress(u,0,.3f)*(1-lateral_instep_progress(u,.6f,1));
            float hip=lateral_instep_progress(u,cfg[5],cfg[5]+.3f)*(1-lateral_instep_progress(u,.6f,1));
            q[1]+=(folded[1]-q[1])*hip;
            q[2]+=(folded[2]-q[2])*knee;
            arc_foot(i,q,goal,NULL);
        }
        /* Once the left pair is down, right J1 stays at its push endpoint
         * until the right lift phase; it never adds another grounded push. */
        goal[1]=base_y+scale*(left?travel-body:
            p<=duration?-body:-push*(1-lateral_instep_progress(u,.35f,.8f)));
        if(!arc_swing_shape_refine(i,goal,q))return false;
        target[i]=(GaitPolicyLegTarget){q[0],q[1],q[2],u<=0 || u>=1};
    }
    return true;
}
static inline bool lateral_instep_targets(float phase,float scale,float input,
        const uint32_t lift_mm[4],GaitPolicyLegTarget target[4]) {
    const float cfg[6]={.035f,.035f,.2f,1.f,.1f,.12f};
    return lateral_instep_configured(phase,scale,input,lift_mm,cfg,target);
}
#endif

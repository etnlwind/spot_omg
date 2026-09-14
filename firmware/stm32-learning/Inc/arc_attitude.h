#ifndef ARC_ATTITUDE_H
#define ARC_ATTITUDE_H
#include "arc_turn.h"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "fp-contract=off")
#endif

/* Opt-in CAD position correction, not force/contact control. Angles and rates
 * come from the delayed IMU; the target phase already includes any trajectory
 * lead. Stance XY stays fixed. A support foot moves vertically with the body
 * tilt to turn the supported body back; a swing foot receives the opposite
 * correction to preserve its height in the level ground frame.
 *
 * Each role fades to zero at the contact schedule boundaries. No support force
 * is inferred from simulator contacts. The caller must validate tilt/sensor
 * safety independently, and pass the ACTUAL gait period including speed scale.
 */
typedef struct { float dz[4]; } ArcAttitudeState;
typedef struct {
    float support_gain, swing_gain, derivative_s;
    float max_correction_m, slew_m_s, transition_s;
} ArcAttitudeConfig;

static inline float arc_attitude_envelope(float phase,float duty,float period,float transition_s) {
    float u=phase-floorf(phase);
    float duration=u<duty?duty:1.f-duty;
    float local=u<duty?u:u-duty;
    float edge=fminf(transition_s/period,.5f*duration);
    if(edge<=0)return 1.f;
    return gait_policy_smootherstep(gait_policy_clampf(local/edge,0,1)) *
        gait_policy_smootherstep(gait_policy_clampf((duration-local)/edge,0,1));
}

static inline bool arc_attitude_apply(ArcAttitudeState *state,
        const GaitPolicyLegTarget nominal[4],float target_phase,float period,float duty,
        float roll,float pitch,float roll_rate,float pitch_rate,bool available,float dt,
        const ArcAttitudeConfig *config,GaitPolicyLegTarget out[4]) {
    if(!state||!nominal||!config||!out)return false;
    if(!isfinite(target_phase)||!isfinite(period)||period<.2f||period>10.f||
       !isfinite(duty)||duty<.5f||duty>.65f||!isfinite(dt)||dt<=0||dt>.06f)return false;
    if(!isfinite(config->support_gain)||config->support_gain<0||config->support_gain>2||
       !isfinite(config->swing_gain)||config->swing_gain<0||config->swing_gain>2||
       !isfinite(config->derivative_s)||config->derivative_s<0||config->derivative_s>.1f||
       !isfinite(config->max_correction_m)||config->max_correction_m<0||config->max_correction_m>.01f||
       !isfinite(config->slew_m_s)||config->slew_m_s<=0||config->slew_m_s>.2f||
       !isfinite(config->transition_s)||config->transition_s<.02f||config->transition_s>.25f)return false;
    if(available&&(!isfinite(roll)||!isfinite(pitch)||!isfinite(roll_rate)||!isfinite(pitch_rate)))return false;
    for(int i=0;i<4;i++){
        if(!isfinite(state->dz[i])||fabsf(state->dz[i])>.010001f||
           !isfinite(nominal[i].j1_deg)||!isfinite(nominal[i].j2_deg)||!isfinite(nominal[i].j3_deg)||
           nominal[i].j1_deg< -30||nominal[i].j1_deg>30||nominal[i].j2_deg< -45||nominal[i].j2_deg>100||
           nominal[i].j3_deg<0||nominal[i].j3_deg>150)return false;
    }
    float r=0,p=0;
    if(available){
        r=gait_policy_clampf(roll+config->derivative_s*roll_rate,-.12f,.12f);
        p=gait_policy_clampf(pitch+config->derivative_s*pitch_rate,-.12f,.12f);
    }
    float sr=sinf(r),cr=cosf(r),sp=sinf(p),cp=cosf(p);
    ArcAttitudeState next=*state;GaitPolicyLegTarget result[4];
    for(int i=0;i<4;i++){
        float q[3]={nominal[i].j1_deg,nominal[i].j2_deg,nominal[i].j3_deg};
        float foot[3];arc_foot(i,q,foot,NULL);
        float u=target_phase+((i==1||i==2)?.5f:0);u-=floorf(u);
        float requested=0;
        if(available){
            float x=foot[0]-arc_center[0],y=foot[1]-arc_center[1],z=foot[2]-arc_center[2];
            float height_error=-sp*x+cp*sr*y+(cp*cr-1.f)*z;
            float role=u<duty?config->support_gain:-config->swing_gain/(cp*cr);
            requested=gait_policy_clampf(role*height_error *
                arc_attitude_envelope(u,duty,period,config->transition_s),
                -config->max_correction_m,config->max_correction_m);
        }
        next.dz[i]+=gait_policy_clampf(requested-next.dz[i],-config->slew_m_s*dt,config->slew_m_s*dt);
        float goal[3]={foot[0],foot[1],foot[2]+next.dz[i]};
        if(fabsf(next.dz[i])>1.e-8f&&!arc_ik(i,goal,q))return false;
        result[i]=nominal[i];result[i].j1_deg=q[0];result[i].j2_deg=q[1];result[i].j3_deg=q[2];
    }
    *state=next;for(int i=0;i<4;i++)out[i]=result[i];
    return true;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

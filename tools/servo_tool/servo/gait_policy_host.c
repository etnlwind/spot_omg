#include "gait_policy.h"
#include "foot_lift.h"

#if defined(_WIN32)
#define SPOT_GAIT_EXPORT __declspec(dllexport)
#else
#define SPOT_GAIT_EXPORT __attribute__((visibility("default")))
#endif

static void unpack_targets(const float values[12],
                           GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        targets[leg].j1_deg = values[leg * 3U];
        targets[leg].j2_deg = values[leg * 3U + 1U];
        targets[leg].j3_deg = values[leg * 3U + 2U];
        targets[leg].stance = false;
    }
}

static void pack_targets(const GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT],
                         float values[12])
{
    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        values[leg * 3U] = targets[leg].j1_deg;
        values[leg * 3U + 1U] = targets[leg].j2_deg;
        values[leg * 3U + 2U] = targets[leg].j3_deg;
    }
}

SPOT_GAIT_EXPORT int spot_gait_trot5_targets(
    float phase, float amplitude_scale, float values[12], uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_trot5_targets(phase, amplitude_scale, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT float spot_gait_smootherstep(float progress)
{
    return gait_policy_smootherstep(progress);
}

SPOT_GAIT_EXPORT int spot_gait_trot_targets(
    float phase,
    float amplitude_scale,
    float travel_scale,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_trot_targets(
            phase, amplitude_scale, travel_scale, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_sim_trot_targets(
    float phase,
    float amplitude_scale,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_sim_trot_targets(
            phase, amplitude_scale, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_trot2_targets(
    float phase,
    float amplitude_scale,
    float fold_j2_deg,
    float fold_j3_deg,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_trot2_targets(
            phase,
            amplitude_scale,
            fold_j2_deg,
            fold_j3_deg,
            targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_trot3_targets(
    float phase,
    float amplitude_scale,
    float fold_j2_deg,
    float fold_j3_deg,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_trot3_targets(
            phase,
            amplitude_scale,
            fold_j2_deg,
            fold_j3_deg,
            targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_trot4_targets(
    float phase,
    float amplitude_scale,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_trot4_targets(phase, amplitude_scale, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_trot4_direction_targets(
    float phase,
    float amplitude_scale,
    int direction,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_trot4_direction_targets(
            phase, amplitude_scale, (int8_t)direction, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_turn_targets(
    float phase,
    float amplitude_scale,
    int direction,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_turn_targets(
            phase, amplitude_scale, (int8_t)direction, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_drive_targets(
    float phase,
    float startup_scale,
    float linear,
    float yaw,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_drive_targets(
            phase, startup_scale, linear, yaw, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_crab_targets(
    float phase,
    float amplitude_scale,
    int direction,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_crab_targets(
            phase, amplitude_scale, (int8_t)direction, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_jump_targets(
    float phase,
    float forward_travel,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_jump_targets(
            phase, forward_travel, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_balance_targets(
    const float imu_values[4],
    const float balance_values[7],
    int contact_aware,
    uint8_t support_mask,
    float values[12])
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (imu_values == NULL || balance_values == NULL || values == NULL) {
        return 0;
    }
    unpack_targets(values, targets);
    const GaitPolicyImuSample sample = {
        imu_values[0], imu_values[1], imu_values[2], imu_values[3]
    };
    const GaitPolicyBalanceConfig config = {
        balance_values[0], balance_values[1], balance_values[2],
        balance_values[3], balance_values[4], balance_values[5],
        balance_values[6], contact_aware != 0
    };
    if (!gait_policy_balance_targets(
            &sample, &config, support_mask, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    return 1;
}

SPOT_GAIT_EXPORT int16_t spot_gait_drive_yaw_limit(int16_t requested)
{
    return gait_policy_drive_yaw_limit(requested);
}

SPOT_GAIT_EXPORT int spot_gait_drive_stride_targets(
    float phase, float startup, float linear, float yaw, float stride,
    float values[12], uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_drive_stride_targets(phase, startup, linear, yaw, stride, targets)) return 0;
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_drive_walk_targets(
    float phase, float startup, float linear, float yaw,
    float values[12], uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_drive_walk_targets(phase, startup, linear, yaw, targets)) return 0;
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int16_t spot_gait_drive_slew(int16_t current, int16_t target)
{
    return gait_policy_drive_slew(current, target);
}
SPOT_GAIT_EXPORT uint16_t spot_gait_drive_period_ms(int16_t linear, int16_t yaw)
{
    return gait_policy_drive_period_ms(linear, yaw);
}

#include "heading_control.h"
SPOT_GAIT_EXPORT float spot_heading_update(float v[6], float heading,
    int valid, int enabled, float linear, float manual_yaw, float applied_yaw, float dt)
{
    HeadingControl s = {v[0], v[1], v[2], v[3], v[4], v[5] != 0};
    float result = heading_update(&s, heading, valid, enabled, linear, manual_yaw, applied_yaw, dt);
    v[0]=s.reference; v[1]=s.error; v[2]=s.integral;
    v[3]=s.correction; v[4]=s.settling; v[5]=s.active;
    return result;
}

#include "locomotion.h"
SPOT_GAIT_EXPORT int spot_locomotion_targets(int profile,float phase,float scale,float linear,float yaw,float values[12]) {
    GaitPolicyLegTarget targets[4];
    if(!locomotion_targets(profile,phase,scale,linear,yaw,targets)) return 0;
    pack_targets(targets,values);return 1;
}
SPOT_GAIT_EXPORT int spot_foot_targets(const float params[7],float phase,float scale,int family,float linear,float yaw,float values[12]) {
    GaitPolicyLegTarget targets[4];
    if(!locomotion_foot_targets(params,phase,scale,family,linear,yaw,targets)) return 0;
    pack_targets(targets,values);return 1;
}
SPOT_GAIT_EXPORT float spot_locomotion_start_scale(int profile,float elapsed,float linear) {return locomotion_start_scale(profile,elapsed,linear);}
SPOT_GAIT_EXPORT float spot_locomotion_period(int profile,float linear,float yaw) {return locomotion_period(profile,linear,yaw);}
SPOT_GAIT_EXPORT void spot_locomotion_params(int profile,float linear,float params[7]) {locomotion_params(profile,linear,params);}

#include "balance_control.h"
SPOT_GAIT_EXPORT int spot_balance_control(float state[16],float values[12],const float imu[4],int enabled,int standing,float kp,float kd,float ki) {
    BalanceControl s={0};s.integral[0]=state[0];s.integral[1]=state[1];
    for(int i=0;i<12;i++)s.correction[i]=state[i+2];
    s.saturated=state[14]!=0;s.applied=state[15]!=0;
    GaitPolicyLegTarget targets[4];unpack_targets(values,targets);
    GaitPolicyImuSample sample={imu[0],imu[1],imu[2],imu[3]};
    if(!balance_control_apply(&s,targets,&sample,enabled,standing,kp,kd,ki))return 0;
    pack_targets(targets,values);
    state[0]=s.integral[0];state[1]=s.integral[1];
    for(int i=0;i<12;i++)state[i+2]=s.correction[i];
    state[14]=s.saturated;state[15]=s.applied;return 1;
}
SPOT_GAIT_EXPORT int spot_balance_control_policy(float state[16],float values[12],const float imu[4],int enabled,int standing,float kp,float kd,float ki,int profile,float phase,int moving,float linear,float yaw) {
    BalanceControl s={0};s.integral[0]=state[0];s.integral[1]=state[1];
    for(int i=0;i<12;i++)s.correction[i]=state[i+2];
    s.saturated=state[14]!=0;s.applied=state[15]!=0;
    GaitPolicyLegTarget targets[4];unpack_targets(values,targets);
    GaitPolicyImuSample sample={imu[0],imu[1],imu[2],imu[3]};
    if(!balance_control_apply_policy(&s,targets,&sample,enabled,standing,kp,kd,ki,profile,phase,moving,linear,yaw))return 0;
    pack_targets(targets,values);
    state[0]=s.integral[0];state[1]=s.integral[1];
    for(int i=0;i<12;i++)state[i+2]=s.correction[i];
    state[14]=s.saturated;state[15]=s.applied;return 1;
}

#include "attitude_control.h"
#include "tilt_pause.h"
SPOT_GAIT_EXPORT int spot_tilt_pause(int v[3],int roll,int pitch,int limit) {
    TiltPause s={v[0],v[1],v[2]!=0};
    int result=tilt_pause_exceeded(&s,roll,pitch,limit);
    v[2]=s.active;return result;
}
#include "drive_control.h"
SPOT_GAIT_EXPORT int spot_attitude_update(int v[9],int valid,int roll,int pitch) {
    AttitudeControl s={{v[0],v[1]},{v[2],v[3]},{v[4],v[5]},v[6],v[7],v[8]!=0};
    int fault=attitude_update(&s,valid,roll,pitch);
    for(int i=0;i<2;i++){v[i]=s.previous[i];v[i+2]=s.filtered[i];v[i+4]=s.rate[i];}
    v[6]=s.failures;v[7]=s.tilt_frames;v[8]=s.initialized;return fault;
}
static int drive_host_step(float v[11],float preload[12],int profile,float linear,float yaw,float heading,int valid,int enabled,int stopping,float dt,float rate,float out[12]) {
    bool navigation=locomotion_is_navigation(profile);
    if((profile==locomotion_profile_id("arcsupport") || navigation) && !preload)return 0;
    DriveControl s={.phase=v[0],.linear=v[1],.yaw=v[2],.elapsed=v[3],.heading={v[4],v[5],v[6],v[7],v[8],v[9]!=0},.turn_assist=v[10]};
    if(preload)for(int i=0;i<12;i++)s.arc_support_preload[i]=preload[i];
    if(navigation){s.lateral=preload[0];s.navigation_period=preload[1];s.side_mode=(int)preload[2];s.heading_direction=(int)preload[3];}
    GaitPolicyLegTarget targets[4];
    if(!drive_control_step_timed(&s,profile,linear,yaw,heading,valid,enabled,stopping,dt,rate,targets))return 0;
    v[0]=s.phase;v[1]=s.linear;v[2]=s.yaw;v[3]=s.elapsed;
    v[4]=s.heading.reference;v[5]=s.heading.error;v[6]=s.heading.integral;v[7]=s.heading.correction;v[8]=s.heading.settling;v[9]=s.heading.active;
    v[10]=s.turn_assist;
    if(preload)for(int i=0;i<12;i++)preload[i]=s.arc_support_preload[i];
    if(navigation){preload[0]=s.lateral;preload[1]=s.navigation_period;preload[2]=s.side_mode;preload[3]=s.heading_direction;}
    pack_targets(targets,out);return 1;
}
SPOT_GAIT_EXPORT int spot_drive_step_timed(float v[11],int profile,float linear,float yaw,float heading,int valid,int enabled,int stopping,float dt,float rate,float out[12]) {
    return drive_host_step(v,NULL,profile,linear,yaw,heading,valid,enabled,stopping,dt,rate,out);
}
SPOT_GAIT_EXPORT int spot_drive_step_stateful(float v[11],float preload[12],int profile,float linear,float yaw,float heading,int valid,int enabled,int stopping,float dt,float rate,float out[12]) {
    return drive_host_step(v,preload,profile,linear,yaw,heading,valid,enabled,stopping,dt,rate,out);
}

SPOT_GAIT_EXPORT int spot_drive_step(float v[11],int profile,float linear,float yaw,float heading,int valid,int enabled,int stopping,float out[12]) {
 return spot_drive_step_timed(v,profile,linear,yaw,heading,valid,enabled,stopping,.02f,1,out);
}
SPOT_GAIT_EXPORT int spot_navigation_pair_extra_lift(const uint32_t mm[4],float phase,float scale,float values[12]) {
    GaitPolicyLegTarget q[4];for(int i=0;i<4;i++)q[i]=(GaitPolicyLegTarget){values[3*i],values[3*i+1],values[3*i+2],false};
    if(!navigation_pair_extra_lift(mm,phase,scale,q))return 0;pack_targets(q,values);return 1;
}
SPOT_GAIT_EXPORT unsigned spot_navigation_size(void){return sizeof(DriveControl);}
SPOT_GAIT_EXPORT int spot_navigation_v10_configured(float phase,float scale,float input,const uint32_t mm[4],const float cfg[6],float values[12]) {
    GaitPolicyLegTarget q[4];for(int i=0;i<4;i++)q[i]=(GaitPolicyLegTarget){values[3*i],values[3*i+1],values[3*i+2],false};
    if(!lateral_instep_configured(phase,scale,input,mm,cfg,q))return 0;
    pack_targets(q,values);return 1;
}
SPOT_GAIT_EXPORT int spot_navigation_v10_targets(float phase,float scale,float input,const uint32_t mm[4],float values[12]) {
    GaitPolicyLegTarget q[4];for(int i=0;i<4;i++)q[i]=(GaitPolicyLegTarget){values[3*i],values[3*i+1],values[3*i+2],false};
    if(!lateral_instep_targets(phase,scale,input,mm,q))return 0;
    pack_targets(q,values);return 1;
}
SPOT_GAIT_EXPORT int spot_navigation_v9_extra_lift(const uint32_t mm[4],float phase,float scale,float values[12]) {
    GaitPolicyLegTarget q[4];for(int i=0;i<4;i++)q[i]=(GaitPolicyLegTarget){values[3*i],values[3*i+1],values[3*i+2],false};
    if(!navigation_pair_extra_lift_offsets(mm,phase,scale,navigation_ipsilateral_offsets,q))return 0;
    pack_targets(q,values);return 1;
}
SPOT_GAIT_EXPORT int spot_navigation_v9_transfer(float phase,float scale,float distance,float values[12]) {
    GaitPolicyLegTarget q[4];for(int i=0;i<4;i++)q[i]=(GaitPolicyLegTarget){values[3*i],values[3*i+1],values[3*i+2],false};
    if(!navigation_ipsilateral_transfer(phase,scale,distance,q))return 0;
    pack_targets(q,values);return 1;
}
SPOT_GAIT_EXPORT int spot_navigation_step(void *state,float linear,float yaw,float heading,
        int valid,int enabled,int stopping,float dt,float rate,float out[12],float diag[16]) {
    DriveControl *s=state;GaitPolicyLegTarget targets[4];
    if(!drive_control_step_timed(s,locomotion_profile_id("attitudepd_v7"),linear,yaw,
            heading,valid,enabled,stopping,dt,rate,targets))return 0;
    pack_targets(targets,out);
    float v[16]={s->phase,s->linear,s->yaw,s->elapsed,s->heading.reference,s->heading.error,
        s->heading.integral,s->heading.correction,s->heading.settling,s->heading.active,
        s->turn_assist,s->lateral,s->side_mode,s->navigation_period,navigation_support_duty(s),s->paired_side && s->side_mode==1?NAVIGATION_PAIR_DUTY:0};
    memcpy(diag,v,sizeof v);return 1;
}
SPOT_GAIT_EXPORT int spot_navigation_v8_step(void *state,float linear,float yaw,float heading,
        int valid,int enabled,int stopping,float dt,float rate,float out[12],float diag[16]) {
    DriveControl *s=state;GaitPolicyLegTarget targets[4];
    if(!drive_control_step_timed(s,locomotion_profile_id("attitudepd_v8"),linear,yaw,
            heading,valid,enabled,stopping,dt,rate,targets))return 0;
    pack_targets(targets,out);
    float v[16]={s->phase,s->linear,s->yaw,s->elapsed,s->heading.reference,s->heading.error,
        s->heading.integral,s->heading.correction,s->heading.settling,s->heading.active,
        s->turn_assist,s->lateral,s->side_mode,s->navigation_period,navigation_support_duty(s),s->paired_side && s->side_mode==1?NAVIGATION_PAIR_DUTY:0};
    memcpy(diag,v,sizeof v);return 1;
}
SPOT_GAIT_EXPORT int spot_navigation_v9_step(void *state,float linear,float yaw,float heading,
        int valid,int enabled,int stopping,float dt,float rate,float out[12],float diag[16]) {
    DriveControl *s=state;GaitPolicyLegTarget targets[4];
    if(!drive_control_step_timed(s,locomotion_profile_id("attitudepd_v9"),linear,yaw,
            heading,valid,enabled,stopping,dt,rate,targets))return 0;
    pack_targets(targets,out);
    float v[16]={s->phase,s->linear,s->yaw,s->elapsed,s->heading.reference,s->heading.error,
        s->heading.integral,s->heading.correction,s->heading.settling,s->heading.active,
        s->turn_assist,s->lateral,s->side_mode,s->navigation_period,navigation_support_duty(s),s->paired_side && s->side_mode==1?NAVIGATION_PAIR_DUTY:0};
    memcpy(diag,v,sizeof v);return 1;
}
SPOT_GAIT_EXPORT int spot_navigation_v10_step(void *state,float linear,float yaw,float heading,
        int valid,int enabled,int stopping,float dt,float rate,float out[12],float diag[16]) {
    DriveControl *s=state;GaitPolicyLegTarget targets[4];
    if(!drive_control_step_timed(s,locomotion_profile_id("attitudepd_v10"),linear,yaw,
            heading,valid,enabled,stopping,dt,rate,targets))return 0;
    pack_targets(targets,out);
    float v[16]={s->phase,s->linear,s->yaw,s->elapsed,s->heading.reference,s->heading.error,
        s->heading.integral,s->heading.correction,s->heading.settling,s->heading.active,
        s->turn_assist,s->lateral,s->side_mode,s->navigation_period,navigation_support_duty(s),s->side_mode==1?.8f:0};
    memcpy(diag,v,sizeof v);return 1;
}
#include "gait_tracking.h"
SPOT_GAIT_EXPORT unsigned spot_tracking_size(void){return sizeof(GaitTracking);}
SPOT_GAIT_EXPORT void spot_tracking_reset(void *s,uint32_t now){gait_tracking_reset(s,now);}
SPOT_GAIT_EXPORT void spot_tracking_sample(void *s,unsigned i,float error,uint32_t now){gait_tracking_sample(s,i,error,now);}
SPOT_GAIT_EXPORT float spot_tracking_step(void *s,uint32_t now,float dt,int stop,float diag[4]) {
 GaitTracking *t=s;float rate=gait_tracking_step(t,now,dt,stop!=0);
 diag[0]=t->peak_error;diag[1]=t->oldest_ms;diag[2]=t->fault;diag[3]=t->blocked_ms;return rate;
}
SPOT_GAIT_EXPORT float spot_tracking_step_responsive(void *s,uint32_t now,float dt,int stop,float diag[4]) {
 GaitTracking *t=s;float rate=gait_tracking_step_policy(t,now,dt,stop!=0,true);
 diag[0]=t->peak_error;diag[1]=t->oldest_ms;diag[2]=t->fault;diag[3]=t->blocked_ms;return rate;
}
SPOT_GAIT_EXPORT float spot_tracking_step_rates(void *s,uint32_t now,float dt,int stop,float deceleration,float recovery,float diag[4]) {
 GaitTracking *t=s;float rate=gait_tracking_step_rates(t,now,dt,stop!=0,deceleration,recovery);
 diag[0]=t->peak_error;diag[1]=t->oldest_ms;diag[2]=t->fault;diag[3]=t->blocked_ms;return rate;
}

#include "locomotion_servo.h"
#include "s_native_servo.h"
SPOT_GAIT_EXPORT int spot_native_servo_encode(const float values[12],uint16_t ticks[12],float decoded[12]) {
    GaitPolicyLegTarget targets[4];unpack_targets(values,targets);
    if(!s_native_servo_targets(targets,ticks))return 0;
    for(int i=0;i<12;i++) {
        const RobotJointConfig *j=&g_robot_joints[i];
        float motor=((int)ticks[i]-j->center)*j->direction*360.f/4096.f;
        /* Physical front J1 rotates opposite to the CAD angle convention. */
        decoded[j->leg_index*3+j->joint_index-1]=j->leg_index<2 && j->joint_index==1?-motor:motor;
    }
    return 1;
}
SPOT_GAIT_EXPORT int spot_servo_encode(const float values[12],uint16_t ticks[12],float decoded[12]) {
    GaitPolicyLegTarget targets[4];unpack_targets(values,targets);
    if(!locomotion_servo_targets(targets,ticks))return 0;
    for(int i=0;i<12;i++) {
        const RobotJointConfig *j=&g_robot_joints[i];
        decoded[j->leg_index*3+j->joint_index-1]=((int)ticks[i]-j->center)*j->direction*360.f/4096.f;
    }
    return 1;
}

#include "stow_control.h"
SPOT_GAIT_EXPORT int spot_stow_frame(const float from[12],int folded,unsigned elapsed,
                                    int32_t ticks[12],float degrees[12]) {
    return stow_frame(from,folded!=0,elapsed,ticks,degrees);
}
SPOT_GAIT_EXPORT int spot_stow_direct(const float from[12],unsigned elapsed,int32_t ticks[12],float degrees[12]) {return stow_direct_frame(from,false,elapsed,ticks,degrees);}
SPOT_GAIT_EXPORT unsigned spot_stow_duration(const float from[12],int folded) {
    return stow_path_duration(from,folded!=0);
}
SPOT_GAIT_EXPORT int spot_stow_geometry(const float q[12]) {return stow_folded_geometry(q);}
SPOT_GAIT_EXPORT int spot_stow_encode(const float degrees[12],int32_t ticks[12],float actual[12]) {
    for(unsigned i=0;i<12;i++) {
        if(!stow_encode(i,degrees[i],&ticks[i]))return 0;
        actual[i]=(ticks[i]-g_robot_joints[i].center)*g_robot_joints[i].direction*360.f/4096.f;
    }
    return 1;
}

SPOT_GAIT_EXPORT int spot_stow_attitude(int valid,int roll,int pitch){return stow_attitude_ok(valid,roll,pitch);}

#include "pose_control.h"
SPOT_GAIT_EXPORT int spot_pose_frame(const float start[12],const float end[12],unsigned elapsed,int stand_requested,float out[12]) {
    uint16_t from[12],to[12],ticks[12];
    for(unsigned i=0;i<12;i++) {
        if(!isfinite(start[i]) || !isfinite(end[i]) || fabsf(start[i])>300 || fabsf(end[i])>300)return -1;
        if(!robot_angle_tenths_to_position(i,lroundf(start[i]*10),from+i) || !robot_angle_tenths_to_position(i,lroundf(end[i]*10),to+i))return -1;
    }
    unsigned duration=pose_transition_duration(from,to,stand_requested!=0);pose_frame(from,to,duration,elapsed,ticks);
    for(unsigned i=0;i<12;i++)out[i]=(ticks[i]-g_robot_joints[i].center)*g_robot_joints[i].direction*360.f/4096.f;
    return duration;
}

SPOT_GAIT_EXPORT int spot_arc_configured(float phase,float scale,float linear,float yaw,const float config[4],float values[12]) {
 GaitPolicyLegTarget targets[4];
 if(!arc_turn_targets_configured(phase,scale,linear,yaw,config[0],config[1],config[2],config[3],targets))return 0;
 pack_targets(targets,values);return 1;
}
SPOT_GAIT_EXPORT int spot_arc_trajectory(float phase,float scale,float linear,float yaw,const float config[6],float values[12]) {
 GaitPolicyLegTarget out[4];
 if(!arc_turn_targets_trajectory(phase,scale,linear,yaw,config[0],config[1],config[2],config[3],config[4],config[5],out))return 0;
 pack_targets(out,values);return 1;
}
SPOT_GAIT_EXPORT int spot_arc_tripod(float phase,float scale,float linear,float yaw,const float config[6],float values[12]) {
 const float offsets[4]={0,.5f,.75f,.25f};GaitPolicyLegTarget out[4];
 if(!arc_turn_targets_phased(phase,scale,linear,yaw,config[0],config[1],config[2],config[3],config[4],config[5],offsets,out))return 0;
 pack_targets(out,values);return 1;
}
#include "arc_balance.h"
SPOT_GAIT_EXPORT int spot_arc_balance(const float values[12],float roll,float pitch,float phase,float duty,float gain,float swing_gain,float result[12]) {
 GaitPolicyLegTarget nominal[4],out[4];unpack_targets(values,nominal);
 if(!arc_balance_targets(nominal,roll,pitch,phase,duty,gain,swing_gain,out))return 0;
 pack_targets(out,result);return 1;
}
#include "arc_preload.h"
SPOT_GAIT_EXPORT int spot_arc_preload(float state[12],const float values[12],float phase,float duty,float gain,float stiffness,float result[12]) {
 GaitPolicyLegTarget out[4];unpack_targets(values,out);
 if(!arc_preload_apply(state,out,phase,duty,gain,stiffness))return 0;
 pack_targets(out,result);return 1;
}
SPOT_GAIT_EXPORT void spot_arc_gravity(const float values[12],float bias[12],float com[3]) {
 GaitPolicyLegTarget pose[4];float feet[4][3],jz[4][3];unpack_targets(values,pose);
 arc_gravity_model(pose,(float(*)[3])bias,feet,jz,com);
}
SPOT_GAIT_EXPORT void spot_arc_contact_gravity(const float values[12],float bias[12],float com[3],float feet[12],float jz[12]) {
 GaitPolicyLegTarget pose[4];unpack_targets(values,pose);
 arc_gravity_model(pose,(float(*)[3])bias,(float(*)[3])feet,(float(*)[3])jz,com);
 for(int i=0;i<4;i++){
  float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg},point[3];
  arc_foot_geometry(i,q,point,NULL,&feet[i*3]);
 }
}
#include "arc_support_shift.h"
#include "arc_tripod_support.h"
SPOT_GAIT_EXPORT int spot_arc_tripod_support(float state[2],const float values[12],float phase,float period,float duty,const float config[2],float result[12]) {
 GaitPolicyLegTarget pose[4];unpack_targets(values,pose);
 if(!arc_tripod_support(state,pose,phase,period,duty,config[0],config[1]))return 0;
 pack_targets(pose,result);return 1;
}
SPOT_GAIT_EXPORT int spot_arc_shift(const float values[12],float phase,float width,float gain,float state[2],float result[12]) {
 GaitPolicyLegTarget pose[4];unpack_targets(values,pose);
 if(!arc_support_shift(state,pose,phase,width,gain))return 0;
 pack_targets(pose,result);return 1;
}
SPOT_GAIT_EXPORT int spot_arc_com(const float values[12],float phase,float period,const float imu[4],const float config[4],float state[2],float result[12]) {
 GaitPolicyLegTarget pose[4];unpack_targets(values,pose);
 if(!arc_support_shift_feedback(state,pose,phase,config[0]/period,config[1],imu[0],imu[1],imu[2],imu[3],config[2],config[3]))return 0;
 pack_targets(pose,result);return 1;
}
SPOT_GAIT_EXPORT int spot_arc_posture(const float values[12],const float offset[3],float scale,float result[12]) {
 GaitPolicyLegTarget pose[4];unpack_targets(values,pose);
 if(!isfinite(scale)||scale<0||scale>1)return 0;
 for(int k=0;k<3;k++)if(!isfinite(offset[k])||fabsf(offset[k])>.015f)return 0;
 for(int i=0;i<4;i++){
  float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg},point[3];arc_foot(i,q,point,NULL);
  point[2]-=scale*offset[0];point[0]+=scale*offset[1];point[1]+=scale*offset[2]*(i%2==0?1:-1);
  if(!arc_ik(i,point,q))return 0;
  pose[i].j1_deg=q[0];pose[i].j2_deg=q[1];pose[i].j3_deg=q[2];
 }
 pack_targets(pose,result);return 1;
}
SPOT_GAIT_EXPORT int spot_arc_wave(const float values[12],float phase,float scale,const float params[6],float result[12]) {
 GaitPolicyLegTarget pose[4];unpack_targets(values,pose);
 if(!isfinite(phase)||!isfinite(scale)||scale<0||scale>1)return 0;
 for(int k=0;k<6;k++)if(!isfinite(params[k])||fabsf(params[k])>.01f)return 0;
 float c=cosf(2*GAIT_POLICY_PI*phase),s=sinf(2*GAIT_POLICY_PI*phase);
 float shift[2]={params[0]+params[2]*(c*c-s*s)+params[3]*2*c*s,params[1]+params[4]*c+params[5]*s};
 float norm=sqrtf(shift[0]*shift[0]+shift[1]*shift[1]);
 if(norm>.010001f)return 0;
 for(int k=0;k<2;k++)shift[k]*=scale;
 for(int i=0;i<4;i++){
  float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg},point[3];arc_foot(i,q,point,NULL);
  for(int k=0;k<2;k++)point[k]-=shift[k];
  if(!arc_ik(i,point,q))return 0;
  pose[i].j1_deg=q[0];pose[i].j2_deg=q[1];pose[i].j3_deg=q[2];
 }
 pack_targets(pose,result);return 1;
}

#include "arc_attitude.h"
#include "arc_transfer.h"
SPOT_GAIT_EXPORT int spot_arc_transfer(float state[12],const float values[12],float phase,float period,float duty,const float config[3],float dt,float result[12]) {
 GaitPolicyLegTarget pose[4];unpack_targets(values,pose);
 if(!arc_transfer_apply(state,pose,phase,period,duty,config[0],config[1],config[2],35.f,dt))return 0;
 pack_targets(pose,result);return 1;
}

SPOT_GAIT_EXPORT int spot_arc_attitude(float state[4],const float values[12],float phase,float period,float duty,
        const float imu[4],int available,float dt,const float config[6],float output[12]) {
    GaitPolicyLegTarget nominal[4],result[4];ArcAttitudeState control;
    for(int i=0;i<4;i++){
        control.dz[i]=state[i];
        nominal[i]=(GaitPolicyLegTarget){values[3*i],values[3*i+1],values[3*i+2],false};
    }
    ArcAttitudeConfig parameters={config[0],config[1],config[2],config[3],config[4],config[5]};
    if(!arc_attitude_apply(&control,nominal,phase,period,duty,imu[0],imu[1],imu[2],imu[3],
        available!=0,dt,&parameters,result))return 0;
    for(int i=0;i<4;i++){
        state[i]=control.dz[i];
        output[3*i]=result[i].j1_deg;output[3*i+1]=result[i].j2_deg;output[3*i+2]=result[i].j3_deg;
    }
    return 1;
}

#include "arc_lift_first.h"
SPOT_GAIT_EXPORT int spot_arc_lift_first(const float values[12],float phase,float scale,float linear,float yaw,float duty,float sweep,float fraction,float result[12]) {
 GaitPolicyLegTarget pose[4];unpack_targets(values,pose);
 if(!arc_lift_first_apply(pose,phase,scale,linear,yaw,duty,sweep,fraction))return 0;
 pack_targets(pose,result);return 1;
}
SPOT_GAIT_EXPORT float spot_arc_lift_first_x(float u,float duty,float fraction) {
 return arc_lift_first_x(u,duty,fraction);
}

SPOT_GAIT_EXPORT int spot_foot_lift(const uint32_t mm[4],float phase,float duty,float scale,const float offsets[4],float values[12]) {
 GaitPolicyLegTarget target[4];unpack_targets(values,target);
 if(!foot_lift_apply(mm,phase,duty,scale,offsets,target))return 0;
 pack_targets(target,values);return 1;
}

SPOT_GAIT_EXPORT int spot_foot_width(const int32_t mm[4],float scale,float values[12]) {
 GaitPolicyLegTarget target[4];
 for(int i=0;i<4;i++)target[i]=(GaitPolicyLegTarget){values[i*3],values[i*3+1],values[i*3+2],false};
 if(!foot_width_apply(mm,scale,target))return 0;
 for(int i=0;i<4;i++){values[i*3]=target[i].j1_deg;values[i*3+1]=target[i].j2_deg;values[i*3+2]=target[i].j3_deg;}
 return 1;
}
SPOT_GAIT_EXPORT int spot_foot_width_cad(const int32_t mm[4],float scale,float values[12]) {
    GaitPolicyLegTarget targets[4];unpack_targets(values,targets);
    if(!foot_width_apply_coordinates(mm,scale,true,targets))return 0;
    pack_targets(targets,values);return 1;
}

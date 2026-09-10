#include "gait_policy.h"

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
SPOT_GAIT_EXPORT float spot_locomotion_period(int profile,float linear,float yaw) {return locomotion_period(profile,linear,yaw);}

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
#include "drive_control.h"
SPOT_GAIT_EXPORT int spot_attitude_update(int v[9],int valid,int roll,int pitch) {
    AttitudeControl s={{v[0],v[1]},{v[2],v[3]},{v[4],v[5]},v[6],v[7],v[8]!=0};
    int fault=attitude_update(&s,valid,roll,pitch);
    for(int i=0;i<2;i++){v[i]=s.previous[i];v[i+2]=s.filtered[i];v[i+4]=s.rate[i];}
    v[6]=s.failures;v[7]=s.tilt_frames;v[8]=s.initialized;return fault;
}
SPOT_GAIT_EXPORT int spot_drive_step(float v[10],int profile,float linear,float yaw,float heading,int valid,int enabled,int stopping,float out[12]) {
    DriveControl s={v[0],v[1],v[2],v[3],{v[4],v[5],v[6],v[7],v[8],v[9]!=0}};
    GaitPolicyLegTarget targets[4];
    if(!drive_control_step(&s,profile,linear,yaw,heading,valid,enabled,stopping,targets))return 0;
    v[0]=s.phase;v[1]=s.linear;v[2]=s.yaw;v[3]=s.elapsed;
    v[4]=s.heading.reference;v[5]=s.heading.error;v[6]=s.heading.integral;v[7]=s.heading.correction;v[8]=s.heading.settling;v[9]=s.heading.active;
    pack_targets(targets,out);return 1;
}

#include "locomotion_servo.h"
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
SPOT_GAIT_EXPORT int spot_stow_encode(const float degrees[12],int32_t ticks[12],float actual[12]) {
    for(unsigned i=0;i<12;i++) {
        if(!stow_encode(i,degrees[i],&ticks[i]))return 0;
        actual[i]=(ticks[i]-g_robot_joints[i].center)*g_robot_joints[i].direction*360.f/4096.f;
    }
    return 1;
}

SPOT_GAIT_EXPORT int spot_stow_attitude(int valid,int roll,int pitch){return stow_attitude_ok(valid,roll,pitch);}

#include "pose_control.h"
SPOT_GAIT_EXPORT int spot_pose_frame(const float start[12],const float end[12],unsigned elapsed,float out[12]) {
    uint16_t from[12],to[12],ticks[12];
    for(unsigned i=0;i<12;i++) {
        if(!isfinite(start[i]) || !isfinite(end[i]) || fabsf(start[i])>300 || fabsf(end[i])>300)return -1;
        if(!robot_angle_tenths_to_position(i,lroundf(start[i]*10),from+i) || !robot_angle_tenths_to_position(i,lroundf(end[i]*10),to+i))return -1;
    }
    unsigned duration=pose_duration(from,to);pose_frame(from,to,duration,elapsed,ticks);
    for(unsigned i=0;i<12;i++)out[i]=(ticks[i]-g_robot_joints[i].center)*g_robot_joints[i].direction*360.f/4096.f;
    return duration;
}

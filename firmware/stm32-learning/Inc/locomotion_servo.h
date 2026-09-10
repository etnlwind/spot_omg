#ifndef LOCOMOTION_SERVO_H
#define LOCOMOTION_SERVO_H
#include "gait_policy.h"
#include "robot_config.h"
static inline bool locomotion_servo_targets(const GaitPolicyLegTarget legs[4],uint16_t ticks[12]) {
    if(!legs || !ticks)return false;
    for(int i=0;i<12;i++) {
        const RobotJointConfig *j=&g_robot_joints[i];
        float angles[3]={legs[j->leg_index].j1_deg,legs[j->leg_index].j2_deg,legs[j->leg_index].j3_deg};
        float value=angles[j->joint_index-1]*10;
        if(!isfinite(value) || value < -32767 || value>32767)return false;
        int16_t tenths=(int16_t)(value>=0?value+.5f:value-.5f);
        if(!robot_angle_tenths_to_position(i,tenths,&ticks[i]))return false;
    }
    return true;
}
#endif

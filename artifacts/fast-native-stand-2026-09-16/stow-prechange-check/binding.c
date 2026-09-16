#include <math.h>
#include "pose_control.h"
#define SPOT_GAIT_EXPORT __declspec(dllexport)
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

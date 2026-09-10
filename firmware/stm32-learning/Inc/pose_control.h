#ifndef POSE_CONTROL_H
#define POSE_CONTROL_H
#include <stdint.h>
#include "robot_config.h"
#include <math.h>
static inline uint32_t pose_duration(const uint16_t from[12],const uint16_t to[12]) {
    uint16_t landing[12],stand[12];
    bool fast=robot_landing_targets(landing) && robot_stand_targets(stand);
    for(unsigned i=0;i<12 && fast;i++) {
        int error=(int)from[i]-landing[i];
        if(error< -120 || error>120 || to[i]!=stand[i])fast=false;
    }
    if(fast)return 0; /* Preserve the original Landing -> Stand response. */
    unsigned maximum=0;
    for(unsigned i=0;i<12;i++){unsigned d=from[i]>to[i]?from[i]-to[i]:to[i]-from[i];if(d>maximum)maximum=d;}
    if(maximum<=12)return 0;
    uint32_t ms=maximum*12U;
    if(ms<1500)ms=1500;
    return (ms+19)/20*20;
}
static inline void pose_frame(const uint16_t from[12],const uint16_t to[12],uint32_t duration,uint32_t elapsed,uint16_t out[12]) {
    float t=duration?fminf(1.f,(float)elapsed/duration):1.f;
    float w=t*t*t*(t*(6.f*t-15.f)+10.f);
    for(unsigned i=0;i<12;i++)out[i]=(uint16_t)lroundf(from[i]+((int)to[i]-from[i])*w);
}
#endif

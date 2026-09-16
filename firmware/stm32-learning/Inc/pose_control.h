#ifndef POSE_CONTROL_H
#define POSE_CONTROL_H
#include <stdint.h>
#include "robot_config.h"
#include <math.h>
#define POSE_RECOGNITION_TICKS 80U
static inline bool pose_is_landing(const uint16_t from[12]) {
    uint16_t landing[12];
    if(!from || !robot_landing_targets(landing))return false;
    for(unsigned i=0;i<12;i++) {
        int error=(int)from[i]-landing[i];
        if(error< -(int)POSE_RECOGNITION_TICKS || error>(int)POSE_RECOGNITION_TICKS)return false;
    }
    return true;
}
static inline bool pose_fast_stand(const uint16_t from[12],bool stand_requested) {
    /* The command identifies Stand. Its target is owned by the selected gait;
     * target equality must not decide how quickly a Stand command executes. */
    return stand_requested && pose_is_landing(from);
}
static inline uint32_t pose_duration(const uint16_t from[12],const uint16_t to[12]) {
    unsigned maximum=0;
    for(unsigned i=0;i<12;i++){unsigned d=from[i]>to[i]?from[i]-to[i]:to[i]-from[i];if(d>maximum)maximum=d;}
    if(maximum<=12)return 0;
    uint32_t ms=maximum*12U;
    if(ms<1500)ms=1500;
    return (ms+19)/20*20;
}
static inline uint32_t pose_transition_duration(const uint16_t from[12],const uint16_t to[12],bool stand_requested) {
    return pose_fast_stand(from,stand_requested)?0:pose_duration(from,to);
}
static inline void pose_frame(const uint16_t from[12],const uint16_t to[12],uint32_t duration,uint32_t elapsed,uint16_t out[12]) {
    float t=duration?fminf(1.f,(float)elapsed/duration):1.f;
    float w=t*t*t*(t*(6.f*t-15.f)+10.f);
    for(unsigned i=0;i<12;i++)out[i]=(uint16_t)lroundf(from[i]+((int)to[i]-from[i])*w);
}
#endif

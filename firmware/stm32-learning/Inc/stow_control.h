#ifndef STOW_CONTROL_H
#define STOW_CONTROL_H
#include "robot_config.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define STOW_DURATION_MS 12000U
#define STOW_FRAME_MS 20U
#define STOW_TILT_TENTHS 300
static inline bool stow_attitude_ok(bool valid,int roll,int pitch) {
    return valid && roll>=-STOW_TILT_TENTHS && roll<=STOW_TILT_TENTHS && pitch>=-STOW_TILT_TENTHS && pitch<=STOW_TILT_TENTHS;
}
/* Exact same bounded path and encoder are used by firmware and MuJoCo. */
static inline float stow_endpoint(unsigned i, bool folded) {
    if(i%3==0)return 0.f;
    if(i%3==2)return folded?4.42f:130.f;
    return folded?(i<6?-254.63f:-80.75f):40.f;
}
static inline bool stow_front(unsigned i) {return i==1 || i==4;}
static inline bool stow_encode(unsigned i,float deg,int32_t *ticks) {
    if(i>=12 || !ticks || !isfinite(deg))return false;
    float lo=i%3==0?-30.f:(i%3==2?-5.f:(stow_front(i)?-275.f:-95.f));
    float hi=i%3==0?30.f:(i%3==2?140.f:60.f);
    if(deg<lo || deg>hi)return false;
    const RobotJointConfig *j=&g_robot_joints[i];
    int32_t tenth=(int32_t)lroundf(deg*10.f);
    int32_t scaled=tenth*4096;
    int32_t raw=j->center+j->direction*((scaled+(scaled>=0?1800:-1800))/3600);
    /* Only the STS3250 FRONT J2 axes cross the single-turn boundary. */
    if((!stow_front(i) && (raw<j->minimum || raw>j->maximum)) || raw < -32767 || raw>32767)return false;
    *ticks=raw;return true;
}
static inline bool stow_decode(unsigned i,int32_t ticks,float *deg) {
    if(i>=12 || !deg)return false;
    float a=(ticks-g_robot_joints[i].center)*g_robot_joints[i].direction*360.f/4096.f;
    if(stow_front(i)) {
        /* Unique branch in the physical Stow envelope, narrower than 360deg.
         * This reconstructs geometry after servo turn counters reset. */
        while(a>60.f)a-=360.f;
        while(a< -275.f)a+=360.f;
    }
    int32_t check;
    if(!stow_encode(i,a,&check))return false;
    *deg=a;return true;
}
static inline uint16_t stow_wire(int32_t ticks) {
    return ticks<0?(uint16_t)((-ticks)|0x8000):(uint16_t)ticks;
}
static inline int32_t stow_unwire(uint16_t word) {
    return word&0x8000?-(int32_t)(word&0x7fff):(int32_t)word;
}
static inline int32_t stow_feedback_error(int32_t actual,int32_t target) {
    int32_t error=(actual-target)%4096;
    if(error>2048)error-=4096;
    if(error< -2048)error+=4096;
    return error;
}
static inline bool stow_frame(const float from[12],bool folded,uint32_t elapsed,
                              int32_t ticks[12],float degrees[12]) {
    if(!from || !ticks || !degrees)return false;
    float t=fminf(1.f,elapsed/(float)STOW_DURATION_MS);
    float w=t*t*t*(t*(6.f*t-15.f)+10.f);
    for(unsigned i=0;i<12;i++) {
        float q=from[i]+(stow_endpoint(i,folded)-from[i])*w;
        if(!stow_encode(i,q,&ticks[i]))return false;
        degrees[i]=(ticks[i]-g_robot_joints[i].center)*g_robot_joints[i].direction*360.f/4096.f;
    }
    return true;
}
#endif

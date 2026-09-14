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
/* Directional recovery, independent of synchronized fold progress. All four
 * hips must agree; knees may settle independently after power removal. */
static inline bool stow_folded_geometry(const float q[12]) {
    if(!q)return false;
    for(unsigned i=0;i<12;i++){int32_t raw;if(!stow_encode(i,q[i],&raw))return false;}
    return q[1]<-90.f && q[4]<-90.f && q[7]<-20.f && q[10]<-20.f;
}
/* Tidy knees at the CURRENT hip angles, without folding hips farther into
 * contact. Each leg gets its own fold fraction; J1 returns gently to neutral.
 * These bounds describe a candidate recovery path, not collision clearance. */
static inline void stow_prepare_target(const float from[12],float target[12]) {
    for(unsigned leg=0;leg<4;leg++) {
        unsigned j=leg*3;
        float f=(from[j+1]-40.f)/(stow_endpoint(j+1,true)-40.f);
        f=fmaxf(0.f,fminf(1.f,f));
        target[j]=0.f;target[j+1]=from[j+1];
        target[j+2]=130.f+(4.42f-130.f)*f;
    }
}
static inline uint32_t stow_path_duration(const float from[12],bool folded) {
    return STOW_DURATION_MS*((!folded && stow_folded_geometry(from))?2U:1U);
}
static inline bool stow_path_frame(const float from[12],bool folded,uint32_t elapsed,bool prepare,
                              int32_t ticks[12],float degrees[12]) {
    if(!from || !ticks || !degrees)return false;
    float start[12],end[12];
    for(unsigned i=0;i<12;i++){start[i]=from[i];end[i]=stow_endpoint(i,folded);}
    if(prepare && !folded && stow_folded_geometry(from)) {
        float ready[12];stow_prepare_target(from,ready);
        if(elapsed<=STOW_DURATION_MS) {
            for(unsigned i=0;i<12;i++)end[i]=ready[i];
        } else {
            for(unsigned i=0;i<12;i++)start[i]=ready[i];
            elapsed-=STOW_DURATION_MS;
        }
    }
    float t=fminf(1.f,elapsed/(float)STOW_DURATION_MS);
    float w=t*t*t*(t*(6.f*t-15.f)+10.f);
    for(unsigned i=0;i<12;i++) {
        float q=start[i]+(end[i]-start[i])*w;
        if(!stow_encode(i,q,&ticks[i]))return false;
        degrees[i]=(ticks[i]-g_robot_joints[i].center)*g_robot_joints[i].direction*360.f/4096.f;
    }
    return true;
}
static inline bool stow_frame(const float from[12],bool folded,uint32_t elapsed,int32_t ticks[12],float degrees[12]) {
    return stow_path_frame(from,folded,elapsed,true,ticks,degrees);
}
static inline bool stow_direct_frame(const float from[12],bool folded,uint32_t elapsed,int32_t ticks[12],float degrees[12]) {
    return stow_path_frame(from,folded,elapsed,false,ticks,degrees);
}
#endif

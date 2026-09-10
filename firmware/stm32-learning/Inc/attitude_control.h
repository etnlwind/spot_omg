#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {int previous[2],filtered[2],rate[2],failures,tilt_frames;bool initialized;} AttitudeControl;
static inline int attitude_clip(int x,int n) {return x < -n ? -n : x > n ? n : x;}
/* Return 0 OK, 1 missing IMU, 2 excessive tilt. */
static inline int attitude_update(AttitudeControl *s,bool valid,int roll,int pitch) {
    if(!valid) {if(s->failures<3)s->failures++;return s->failures>=3?1:0;}
    s->failures=0;int errors[2]={attitude_clip(roll,300),attitude_clip(pitch,300)};
    if(!s->initialized) {s->previous[0]=errors[0];s->previous[1]=errors[1];s->initialized=true;}
    if(roll>120 || roll < -120 || pitch>120 || pitch < -120) {if(s->tilt_frames<2)s->tilt_frames++;} else s->tilt_frames=0;
    for(int i=0;i<2;i++) {
        int rate=attitude_clip((errors[i]-s->previous[i])*50,1200);
        s->previous[i]=errors[i];s->filtered[i]=(3*s->filtered[i]+errors[i])/4;
        s->rate[i]=(3*s->rate[i]+rate)/4;
    }
    return s->tilt_frames>=2?2:0;
}
#endif

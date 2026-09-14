#ifndef GAIT_TRACKING_H
#define GAIT_TRACKING_H
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
/* Supervisory feedback, not a contact detector or extra position gain.
 * Errors are paired with the target active at each individual bus read. */
typedef struct {
    float error[12],rate,peak_error;
    uint32_t sampled_at[12],started_at,blocked_ms,oldest_ms;
    uint16_t seen;
    uint8_t fault; /* 1=stale feedback, 2=persistent tracking failure */
} GaitTracking;
static inline void gait_tracking_reset(GaitTracking *s,uint32_t now) {
    memset(s,0,sizeof(*s));s->rate=1;s->started_at=now;
}
static inline void gait_tracking_sample(GaitTracking *s,unsigned joint,
                                        float error_deg,uint32_t now) {
    if(joint>=12 || !isfinite(error_deg))return;
    s->error[joint]=error_deg;s->sampled_at[joint]=now;s->seen|=(uint16_t)(1U<<joint);
}
static inline float gait_tracking_step(GaitTracking *s,uint32_t now,float dt,bool stopping) {
    if(s->fault)return 0;
    s->peak_error=0;s->oldest_ms=0;
    for(unsigned i=0;i<12;i++){
        uint32_t age=now-((s->seen&(1U<<i))?s->sampled_at[i]:s->started_at);
        if(age>s->oldest_ms)s->oldest_ms=age;
        if(s->seen&(1U<<i))s->peak_error=fmaxf(s->peak_error,fabsf(s->error[i]));
    }
    /* One sweep is 240ms nominal. Never reinterpret an old sample as current. */
    if(s->oldest_ms>600){s->fault=1;return 0;}
    if(stopping){s->blocked_ms=0;return 1;} /* Stop cannot wait for phase readiness. */
    float wanted=fmaxf(0,fminf(1,(14-s->peak_error)/8));
    if(s->oldest_ms>360)wanted=0;
    if(wanted==0)s->rate=0;
    else s->rate+=fmaxf(-.6f*dt,fminf(.2f*dt,wanted-s->rate));
    if(s->rate<.05f)s->blocked_ms+=(uint32_t)(dt*1000+.5f);else s->blocked_ms=0;
    if(s->blocked_ms>=600){s->fault=2;s->rate=0;}
    return s->rate;
}
#endif

#ifndef ARC_OBSERVER_H
#define ARC_OBSERVER_H
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* Experimental gait observer only. It never replaces attitude_update's
 * independent missing-IMU/tilt safety checks. Angles are degrees; rates deg/s.
 * Timestamps refer to the original BNO sample, not the control poll time. */
typedef struct {
    float angle_deg[2], rate_deg_s[2], previous_deg[2];
    double sampled_s, updated_s;
    bool initialized, clock_initialized, available;
} ArcObserver;

static inline void arc_observer_reset(ArcObserver *s) {
    if(s)memset(s,0,sizeof(*s));
}

static inline float arc_observer_clip_rate(float rate) {
    return fminf(120.f,fmaxf(-120.f,rate));
}

static inline void arc_observer_missing(ArcObserver *s,double now_s) {
    if(!s)return;
    s->available=false;
    if(!isfinite(now_s)||now_s<0)return;
    if(s->clock_initialized&&now_s>=s->updated_s){
        float decay=expf(-(float)(now_s-s->updated_s)/.1f);
        for(int i=0;i<2;i++){
            s->angle_deg[i]*=decay;s->rate_deg_s[i]*=decay;
        }
    }
    s->updated_s=now_s;s->clock_initialized=true;
}

/* False means unusable input; available is also cleared in that case. Missing
 * samples decay the retained estimate but NEVER assert that the body is level.
 * Stale, future, nonfinite and backward samples are treated as unavailable.
 * Re-polling one sample does not filter it twice. */
static inline bool arc_observer_update(ArcObserver *s,bool valid,float roll_deg,
        float pitch_deg,double sampled_s,double now_s) {
    if(!s)return false;
    if(!isfinite(now_s)||now_s<0||
       (s->clock_initialized&&now_s<s->updated_s)){
        s->available=false;return false;
    }
    if(!valid){arc_observer_missing(s,now_s);return false;}
    if(!isfinite(roll_deg)||!isfinite(pitch_deg)||fabsf(roll_deg)>180.f||fabsf(pitch_deg)>180.f||
       !isfinite(sampled_s)||sampled_s<0||sampled_s>now_s+1.e-6||now_s-sampled_s>.100001||
       (s->initialized&&sampled_s<s->sampled_s)){
        arc_observer_missing(s,now_s);return false;
    }
    float angles[2]={roll_deg,pitch_deg};
    if(s->initialized&&sampled_s==s->sampled_s){
        /* Once this old sample has been invalidated, only a fresh sample can
         * restore availability; a cached read is not recovery evidence. */
        s->updated_s=now_s;s->clock_initialized=true;return s->available;
    }
    float dt=s->initialized?(float)(sampled_s-s->sampled_s):0;
    if(!s->initialized||dt>.100001f){
        for(int i=0;i<2;i++){s->angle_deg[i]=angles[i];s->rate_deg_s[i]=0;}
    }else{
        /* At 20ms: angle alpha .6, rate alpha .25. Exponential conversion
         * preserves these time constants if BNO sample spacing changes. */
        float alpha=-expm1f(-45.8145366f*dt);
        float rate_alpha=-expm1f(-14.3841036f*dt);
        for(int i=0;i<2;i++){
            float rate=arc_observer_clip_rate((angles[i]-s->previous_deg[i])/dt);
            s->angle_deg[i]+=alpha*(angles[i]-s->angle_deg[i]);
            s->rate_deg_s[i]+=rate_alpha*(rate-s->rate_deg_s[i]);
        }
    }
    for(int i=0;i<2;i++)s->previous_deg[i]=angles[i];
    s->sampled_s=sampled_s;s->updated_s=now_s;
    s->initialized=s->clock_initialized=s->available=true;
    return true;
}

/* Prediction is explicit: sensor age is NOT silently added to lookahead.
 * The caller owns that choice and may request at most 100ms. Outputs are
 * untouched when unavailable/invalid so callers must check the return value. */
static inline bool arc_observer_read(const ArcObserver *s,double now_s,
        float lookahead_s,float out[4]) {
    if(!s||!out||!s->initialized||!s->available||!isfinite(now_s)||
       now_s<s->updated_s||now_s-s->sampled_s>.100001||
       !isfinite(lookahead_s)||lookahead_s<0||lookahead_s>.1f)return false;
    for(int i=0;i<2;i++){
        if(!isfinite(s->angle_deg[i])||!isfinite(s->rate_deg_s[i]))return false;
    }
    for(int i=0;i<2;i++){
        out[i]=s->angle_deg[i]+lookahead_s*s->rate_deg_s[i];
        out[i+2]=s->rate_deg_s[i];
    }
    return true;
}
#endif

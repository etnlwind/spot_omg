#ifndef PROBE_CONFIG_H
#define PROBE_CONFIG_H
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
typedef struct { uint16_t lift_mm, linear, duration_ms; uint8_t leg; int16_t width_mm; uint16_t fr_extra; } ProbeConfig;
static inline ProbeConfig probe_config_default(void) {
    return (ProbeConfig){20,344,4000,0,0,0};
}
static inline bool probe_uint(const char *s,unsigned lo,unsigned hi,uint16_t *out) {
    if(!s || !*s)return false;
    unsigned v=0;
    for(;*s;s++){if(*s<'0'||*s>'9')return false;v=v*10+(unsigned)(*s-'0');if(v>hi)return false;}
    if(v<lo)return false;
    *out=(uint16_t)v;return true;
}
/* All fields commit atomically only after complete validation. */
static inline bool probe_config_parse(ProbeConfig *out,const char *lift,const char *linear,
                                      const char *duration,const char *leg) {
    ProbeConfig next={0};
    if(!probe_uint(lift,12,40,&next.lift_mm) || !probe_uint(linear,1,1000,&next.linear) ||
       !probe_uint(duration,500,30000,&next.duration_ms) || !leg)return false;
    if(!strcmp(leg,"all"))next.leg=0;
    else if(!strcmp(leg,"rl"))next.leg=3;
    else if(!strcmp(leg,"rr"))next.leg=4;
    else return false;
    *out=next;return true;
}

/* Signed per-foot offset from the vertical J1 reference. */
static inline bool probe_width_parse(const char *text,int16_t *out) {
    if(!text){*out=0;return true;}
    bool negative=*text=='-';
    if(negative || *text=='+')text++;
    uint16_t value;
    if(!probe_uint(text,0,negative?40:20,&value))return false;
    *out=negative?-(int16_t)value:(int16_t)value;return true;
}
#endif

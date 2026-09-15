#ifndef JOINT_TRACE_H
#define JOINT_TRACE_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define JOINT_TRACE_CAPACITY 256U
#define JOINT_TRACE_SAMPLE_CAPACITY 512U
/* Read-only RAM flight recorder. No bus I/O, allocation, printf or actuation.
 * Freeze on capacity instead of silently overwriting the command history.
 * Time intervals bound transport uncertainty; they are not servo sample times. */
typedef struct {
    uint32_t begin_ms,end_ms;
    uint16_t target[12];
} JointTraceCommand;
typedef struct {
    uint32_t begin_ms,end_ms;
    uint16_t command_index,position,voltage_mv;
    int16_t speed,load,current;
    uint8_t joint,status,temperature,hardware_error;
} JointTraceSample;
typedef struct {
    JointTraceCommand commands[JOINT_TRACE_CAPACITY];
    JointTraceSample samples[JOINT_TRACE_SAMPLE_CAPACITY];
    uint16_t command_count,sample_count,speed;
    uint8_t profile,acceleration;
    bool armed,full,arm_on_stop;
} JointTrace;
static inline void joint_trace_arm(JointTrace *t) {
    t->command_count=t->sample_count=0;t->full=false;t->armed=true;t->arm_on_stop=false;
}
static inline void joint_trace_command(JointTrace *t,uint32_t begin,uint32_t end,const uint16_t target[12]) {
    if(!t->armed)return;
    if(t->command_count>=JOINT_TRACE_CAPACITY){t->armed=false;t->full=true;return;}
    JointTraceCommand *c=&t->commands[t->command_count++];
    c->begin_ms=begin;c->end_ms=end;memcpy(c->target,target,sizeof(c->target));
}
static inline void joint_trace_sample(JointTrace *t,JointTraceSample sample) {
    if(!t->armed||!t->command_count||sample.joint>=12)return;
    if(t->sample_count>=JOINT_TRACE_SAMPLE_CAPACITY){t->armed=false;t->full=true;return;}
    sample.command_index=t->command_count-1;
    t->samples[t->sample_count++]=sample;
}
#endif

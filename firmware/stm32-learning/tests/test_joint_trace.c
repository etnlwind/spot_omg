#include "joint_trace.h"
#include <assert.h>
int main(void){
 static JointTrace t;uint16_t targets[12];for(unsigned i=0;i<12;i++)targets[i]=2000+i;
 JointTraceSample s={.begin_ms=12,.end_ms=18,.joint=5,.position=1999};
 joint_trace_sample(&t,s);assert(t.sample_count==0);
 joint_trace_arm(&t);joint_trace_sample(&t,s);assert(t.sample_count==0);
 joint_trace_command(&t,10,11,targets);targets[5]=0;
 joint_trace_sample(&t,s);assert(t.samples[0].command_index==0);assert(t.commands[0].target[5]==2005);
 s.status=3;joint_trace_sample(&t,s);assert(t.samples[1].status==3);
 while(t.command_count<JOINT_TRACE_CAPACITY)joint_trace_command(&t,20,21,targets);
 joint_trace_command(&t,30,31,targets);assert(t.full&&!t.armed);
 unsigned n=t.sample_count;joint_trace_sample(&t,s);assert(t.sample_count==n);
 assert(t.commands[0].begin_ms==10);joint_trace_arm(&t);
 assert(!t.full&&t.armed&&!t.command_count&&!t.sample_count);
 joint_trace_command(&t,0xfffffff0U,0xfffffff1U,targets);
 s.begin_ms=2;s.end_ms=5;joint_trace_sample(&t,s);assert(t.samples[0].begin_ms==2);
 return 0;
}

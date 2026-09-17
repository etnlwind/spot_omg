"""Exercise production phase-limit block without motors or transport."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[3]

def test_eight_nominal_placements_then_return(tmp_path):
    text=(ROOT/'firmware/stm32-learning/Src/robot.c').read_text()
    a=text.index('        if(robot->gait_step_limit && stage==2 && !stopping)')
    b=text.index('        if(robot->gait_step_limit && (uint32_t)',a)
    block=text[a:b]
    code='''#include <assert.h>
#include "attitude_pd.h"
typedef struct {int gait_step_limit,gait_steps_completed;AttitudePd attitude_pd;} Robot;
int main(void) {
 for(int dt=10;dt<=60;dt+=10) {
  Robot storage={0};Robot *robot=&storage;robot->gait_step_limit=8;
  unsigned step_half=0;int stage=2;bool stopping=false;float transition=1;
  GaitPolicyLegTarget command[4]={{0}},from[4];
  float phase=0;int commanded_eighth=0;
  for(int frame=0;frame<2000 && stage==2;frame++) {
   float target_phase=phase;
'''+block+'''
   if(stage==2 && robot->gait_steps_completed==8)commanded_eighth++;
   phase=fmodf(phase+dt/1350.f,1.f);
  }
  assert(commanded_eighth==1 && robot->gait_steps_completed==8);
  assert(stage==3 && stopping && transition==0);
 }
}
'''
    c=tmp_path/'counter.c';c.write_text(code);exe=tmp_path/'counter'
    subprocess.run(['clang','-O1','-I',str(ROOT/'firmware/stm32-learning/Inc'),str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

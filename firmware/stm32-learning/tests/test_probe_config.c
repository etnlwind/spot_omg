#include <assert.h>
#include "probe_config.h"
int main(void) {
 ProbeConfig p=probe_config_default();
 assert(p.lift_mm==20 && p.duration_ms==4000 && p.linear==344 && p.leg==0);
 assert(probe_config_parse(&p,"40","1000","30000","rr"));
 assert(p.lift_mm==40 && p.linear==1000 && p.duration_ms==30000 && p.leg==4);
 const char *bad[]={"-1","0","11","41","20x","","999999999999999999999999","+20"," 20"};
 for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) {
  assert(!probe_config_parse(&p,bad[i],"344","4000","all"));assert(p.lift_mm==40 && p.leg==4);
 }
 assert(!probe_config_parse(&p,"20","1001","4000","all"));
 assert(!probe_config_parse(&p,"20","344","30001","all"));
 assert(!probe_config_parse(&p,"20","344","4000","fl"));
 assert(!probe_config_parse(&p,0,"344","4000","all"));
 assert(probe_config_parse(&p,"12","1","500","rl"));
 assert(p.leg==3 && p.lift_mm==12);
 return 0;
}

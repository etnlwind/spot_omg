#include <assert.h>
#include <math.h>
#include "probe_config.h"
#include "s_native_impl.h"
int main(void){
 int16_t w;assert(probe_width_parse("-38",&w)&&w==-38);assert(probe_width_parse("0",&w)&&w==0);assert(probe_width_parse("+20",&w)&&w==20);
 assert(!probe_width_parse("-41",&w));assert(!probe_width_parse("21",&w));assert(!probe_width_parse("--1",&w));assert(!probe_width_parse("NaN",&w));
 for(int enabled=0;enabled<2;enabled++) {
  SNativeControl s;s_native_reset_v625(&s);s.diagnostic_leg=5;s.diagnostic_width_active=true;s.diagnostic_width_m=-.02f;s.diagnostic_fr_extra=enabled;
  for(int k=0;k<15;k++) {GaitPolicyLegTarget out[4];assert(s_native_step(&s,.6f,1,.344f,0,.344f,0,.02f,false,out));
   if(enabled){assert(fabsf(out[0].j1_deg-sn_standing[0][0])<.001f);assert(fabsf(out[2].j1_deg-sn_standing[2][0])<.001f);}
  }
 }
 for(int width=-40;width<=20;width+=10){
  SNativeControl s;s_native_reset_v625(&s);s.diagnostic_leg=5;s.diagnostic_lift_extra_m=.016f;s.diagnostic_width_active=true;s.diagnostic_width_m=width*.001f;
  for(int k=0;k<300;k++){
   float phase=fmodf(.5f+k*.01f,1);GaitPolicyLegTarget out[4];assert(s_native_step(&s,phase,1,.344f,0,.344f,0,.02f,false,out));
   for(int i=0;i<4;i++){float q[3]={out[i].j1_deg,out[i].j2_deg,out[i].j3_deg},p[3];s_native_foot(i,q,p,NULL);
    float expected=sn_origin[i][1]+(i%2==0?1:-1)*width*.001f*smooth(s.entry_phase/.5f);assert(fabsf(p[1]-expected)<.001f);
   }
  }
  for(int k=0;k<200;k++){GaitPolicyLegTarget out[4];assert(s_native_step(&s,.49f,1,.344f,0,0,0,.02f,true,out));}
  assert(s_native_stopped(&s));
 }
}

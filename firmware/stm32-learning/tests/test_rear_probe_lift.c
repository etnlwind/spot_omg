#include <assert.h>
#include <math.h>
#include "s_native.h"
#include "s_native_impl.h"
int main(void) {
 for(unsigned leg=3;leg<=5;leg++)for(int mm=20;mm<=36;mm+=8) {
  SNativeControl a,b;s_native_reset_v625(&a);s_native_reset_v625(&b);
  b.diagnostic_leg=leg;b.diagnostic_lift_extra_m=(mm-12)*.001f;
  float peak=0;
  for(int k=0;k<200;k++) {
   float phase=fmodf(.5f+k*.01f,1);GaitPolicyLegTarget x[4],y[4];
   assert(s_native_step(&a,phase,1,.344f,0,.344f,0,.02f,0,x));
   assert(s_native_step(&b,phase,1,.344f,0,.344f,0,.02f,0,y));
   for(int i=0;i<4;i++) {
    float qa[3]={x[i].j1_deg,x[i].j2_deg,x[i].j3_deg},qb[3]={y[i].j1_deg,y[i].j2_deg,y[i].j3_deg},pa[3],pb[3],j[3][3];
    s_native_foot(i,qa,pa,j);s_native_foot(i,qb,pb,j);
    assert(fabsf(qa[0]-qb[0])<.00001f);
    assert(fabsf(pa[0]-pb[0])<.0002f);
    float swing=(fmodf(phase+((i==0||i==3)?.5f:0),1)-.5f)/.5f;
    float expected=(leg==5 || i+1==(int)leg)?(mm-12)*.001f*smooth(fminf(swing,1-swing)/.3f):0;
    assert(fabsf(pb[2]-pa[2]-expected)<.0002f);
    peak=fmaxf(peak,pb[2]-pa[2]);
   }
  }
  assert(peak>(mm-12)*.001f-.0002f);
  for(int k=0;k<200;k++){GaitPolicyLegTarget y[4];assert(s_native_step(&b,.49f,1,.344f,0,0,0,.02f,1,y));}
  assert(s_native_stopped(&b));
 }
 return 0;
}

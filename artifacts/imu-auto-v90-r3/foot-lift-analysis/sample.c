#include <stdio.h>
#include "foot_lift.h"
int main(void){int p=locomotion_profile_id("attitudepd_v4");
 for(int d=-1;d<=1;d+=2)for(int speed=0;speed<2;speed++){
 float v=d*(speed?1.f:.588f);float lo[4]={9,9,9,9},hi[4]={-9,-9,-9,-9},extra[4]={0};int fail=0;
 for(int t=0;t<1000;t++){float phase=t*.001f;GaitPolicyLegTarget a[4],b[4];uint32_t mm[4]={30,30,0,0};
 if(!locomotion_targets(p,phase,1,v,0,a)){fail++;continue;}memcpy(b,a,sizeof b);
 if(!foot_lift_profile(mm,p,phase,1,v,0,b)){fail++;continue;}
 for(int i=0;i<4;i++){float qa[3]={a[i].j1_deg,a[i].j2_deg,a[i].j3_deg},qb[3]={b[i].j1_deg,b[i].j2_deg,b[i].j3_deg},fa[3],fb[3];arc_foot(i,qa,fa,0);arc_foot(i,qb,fb,0);lo[i]=fminf(lo[i],fb[2]);hi[i]=fmaxf(hi[i],fb[2]);extra[i]=fmaxf(extra[i],fb[2]-fa[2]);}}
 printf("linear %.3f failures %d\n",v,fail);for(int i=0;i<4;i++)printf("leg %d z_span_mm %.3f added_peak_mm %.3f\n",i,1000*(hi[i]-lo[i]),1000*extra[i]);}
}

#include "arc_preload.h"
void evaluate(const float *values,float *bias,float *feet,float *jz,float *com){GaitPolicyLegTarget p[4];for(int i=0;i<4;i++){p[i].j1_deg=values[3*i];p[i].j2_deg=values[3*i+1];p[i].j3_deg=values[3*i+2];}arc_gravity_model(p,(float(*)[3])bias,(float(*)[3])feet,(float(*)[3])jz,com);}

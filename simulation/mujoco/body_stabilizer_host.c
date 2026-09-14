#include "attitude_pd.h"
#include <stdlib.h>
#include <stdio.h>

void *spot_pd_create(void) { AttitudePd *p=calloc(1,sizeof(AttitudePd));if(p)body_stabilizer_reset(&p->body);return p; }
void spot_pd_destroy(void *p) {free(p);}
void spot_pd_reset(void *p) {memset(p,0,sizeof(AttitudePd));body_stabilizer_reset(&((AttitudePd*)p)->body);}
size_t spot_pd_config_size(void) {return sizeof(BodyStabilizerConfig);}
void spot_pd_default(BodyStabilizerConfig *c) {*c=body_stabilizer_default_config();}
int spot_pd_config_valid(const BodyStabilizerConfig *c) {return body_stabilizer_config_valid(c);}
int spot_pd_apply(AttitudePd *s,const BodyStabilizerConfig *c,const BodyImuState *imu,
        uint32_t now,float dt,int enabled,float phase,float period,float duty,int moving,
        const float nominal[12],float output[12]) {
    GaitPolicyLegTarget input[4],result[4];
    for(int i=0;i<4;i++)input[i]=(GaitPolicyLegTarget){nominal[3*i],nominal[3*i+1],nominal[3*i+2],false};
    if(!attitude_pd_apply(s,c,imu,now,dt,enabled,phase,period,duty,moving,input,result))return 0;
    for(int i=0;i<4;i++){output[3*i]=result[i].j1_deg;output[3*i+1]=result[i].j2_deg;output[3*i+2]=result[i].j3_deg;}
    return 1;
}
void spot_pd_feet(const float q[12],float out[12]) {for(int i=0;i<4;i++)arc_foot(i,q+3*i,out+3*i,NULL);}
static float finite_or_zero(float x) {return isfinite(x)?x:0;}
static int json_array(char *out,int remaining,const char *name,const float *values,int count) {
    int n=snprintf(out,remaining,",\"%s\":[",name);
    for(int i=0;i<count;i++)n+=snprintf(out+n,remaining-n,"%s%.9g",i?",":"",finite_or_zero(values[i]));
    n+=snprintf(out+n,remaining-n,"]");return n;
}
const char *spot_pd_diagnostic(const AttitudePd *s) {
    static char out[4096];const BodyStabilizerDiagnostics *d=&s->body.diagnostics;
    int n=snprintf(out,sizeof(out),"{\"status\":\"%s\",\"sample_age_ms\":%u,\"sample_sequence\":%u,\"sample_timestamp_ms\":%u,\"enabled\":%d,\"new_sample\":%d,\"input_finite\":%d,\"axis_clamp_mask\":%u,\"foot_clamp_mask\":%u,\"slew_limited\":%d,\"ik_failed\":%d,\"max_ik_error_m\":%.9g",
        attitude_pd_status_name(s),d->sample_age_ms,d->sample_sequence,d->sample_timestamp_ms,
        d->enabled,d->new_sample,d->input_finite,d->axis_clamp_mask,d->foot_clamp_mask,d->slew_limited,s->ik_failed,s->max_ik_error_m);
    #define ARRAY(field,count) n+=json_array(out+n,sizeof(out)-n,#field,d->field,count)
    ARRAY(raw,4);ARRAY(filtered,4);ARRAY(error,2);ARRAY(u_raw,2);ARRAY(u_requested,2);
    ARRAY(u_applied,2);ARRAY(stance_weights,4);ARRAY(requested_dz,4);ARRAY(applied_dz,4);
    #undef ARRAY
    snprintf(out+n,sizeof(out)-n,"}");return out;
}

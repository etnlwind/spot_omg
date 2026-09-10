#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H
#include "gait_policy.h"
#include "heading_control.h"
#include "locomotion.h"

typedef struct {float integral[2],correction[12];bool saturated,applied;} BalanceControl;
static inline bool balance_control_apply_internal(BalanceControl *s,GaitPolicyLegTarget target[4],
        const GaitPolicyImuSample *input,bool enabled,bool standing,float kp,float kd,float ki,
        const float support[4],const float placement[4]) {
    GaitPolicyLegTarget corrected[4];
    for(int i=0;i<4;i++) corrected[i]=target[i];
    float limit=standing?.12f:.06f,joint_limit=standing?10.f:6.f;
    float error[2]={input->roll,input->pitch};bool length_sat=false;
    s->applied=enabled;
    if(enabled) {
        for(int i=0;i<2;i++) if(!s->saturated || error[i]*s->integral[i]<0)
            s->integral[i]=heading_clip(s->integral[i]+ki*error[i]*.02f,limit);
        GaitPolicyImuSample controlled=*input;
        controlled.roll+=s->integral[0]/fmaxf(kp,.001f);
        controlled.pitch+=s->integral[1]/fmaxf(kp,.001f);
        length_sat=fabsf(kp*controlled.roll+kd*input->roll_rate)+fabsf(kp*controlled.pitch+kd*input->pitch_rate)>=limit;
        GaitPolicyBalanceConfig config={kp,kd,limit,0,0,0,0,true};
        if(support) {
            /* Predict only a bounded 25 ms from delayed, filtered Euler rates.
             * Phase describes expected support, never a measured contact. */
            controlled.roll+=heading_clip(input->roll_rate*.025f,.02f);
            controlled.pitch+=heading_clip(input->pitch_rate*.025f,.02f);
            GaitPolicyBalancePreview preview;
            if(!gait_policy_balance_preview(&controlled,&config,15,&preview))return false;
            for(int i=0;i<4;i++) {
                float forward,down;
                gait_policy_leg_forward_kinematics(target[i].j2_deg,target[i].j3_deg,&forward,&down);
                float dz=preview.down_correction[i]*(.65f+.35f*support[i]);
                float dx=heading_clip(.03f*controlled.pitch,.003f)*placement[i];
                float side=i%2==0?1.f:-1.f;
                corrected[i].j1_deg+=heading_clip(5.f*controlled.roll*side*(2.f*support[i]-1.f),1.5f);
                bool reachable=false;float fraction=1.f;
                for(int attempt=0;attempt<8;attempt++,fraction*=.5f) {
                    if(gait_policy_leg_inverse_kinematics(forward+dx*fraction,down+dz*fraction,
                        &corrected[i].j2_deg,&corrected[i].j3_deg)){reachable=true;break;}
                    length_sat=true;
                }
                if(!reachable) {corrected[i]=target[i];length_sat=true;}
            }
        } else if(!gait_policy_balance_targets(&controlled,&config,15,corrected)) return false;
    } else s->integral[0]=s->integral[1]=0;
    s->saturated=length_sat;
    for(int i=0;i<4;i++) {
        float nominal[3]={target[i].j1_deg,target[i].j2_deg,target[i].j3_deg};
        float desired[3]={corrected[i].j1_deg,corrected[i].j2_deg,corrected[i].j3_deg};
        float lo[3]={-30,-45,0},hi[3]={30,100,150};
        for(int j=0;j<3;j++) {
            int index=i*3+j;float delta=desired[j]-nominal[j];
            if(fabsf(delta)>joint_limit)s->saturated=true;
            s->correction[index]+=heading_clip(heading_clip(delta,joint_limit)-s->correction[index],.3f);
            desired[j]=gait_policy_clampf(nominal[j]+s->correction[index],lo[j],hi[j]);
        }
        target[i].j1_deg=desired[0];target[i].j2_deg=desired[1];target[i].j3_deg=desired[2];
    }
    return true;
}
static inline bool balance_control_apply(BalanceControl *s,GaitPolicyLegTarget target[4],
        const GaitPolicyImuSample *input,bool enabled,bool standing,float kp,float kd,float ki) {
    return balance_control_apply_internal(s,target,input,enabled,standing,kp,kd,ki,NULL,NULL);
}
static inline void balance_support_weights(float phase,float duty,float support[4],float placement[4]) {
    const float offsets[4]={0,.5f,.5f,0};
    for(int i=0;i<4;i++) {
        float q=phase+offsets[i];q-=floorf(q);
        float w=1;
        if(q>duty-.08f && q<duty)w=1-gait_policy_smootherstep((q-duty+.08f)/.08f);
        else if(q>=duty)w=gait_policy_smootherstep(gait_policy_clampf((q-.92f)/.08f,0,1));
        support[i]=w;
        /* Retain the swing placement through touchdown, then release it. */
        placement[i]=q<duty?1-gait_policy_smootherstep(fminf(1,q/.12f)):
            gait_policy_smootherstep((q-duty)/(1-duty));
    }
}
static inline bool balance_control_apply_policy(BalanceControl *s,GaitPolicyLegTarget target[4],
        const GaitPolicyImuSample *input,bool enabled,bool standing,float kp,float kd,float ki,
        int profile,float phase,bool moving,float linear,float yaw) {
    if(profile!=locomotion_profile_id("imu") || !moving)
        return balance_control_apply(s,target,input,enabled,standing,kp,kd,ki);
    if(!isfinite(phase)||!isfinite(linear)||!isfinite(yaw))return false;
    float p[7],support[4],placement[4];locomotion_params(profile,linear,p);
    /* drive_control_step advances phase after producing this frame's targets. */
    balance_support_weights(phase-.02f/locomotion_period(profile,linear,yaw),p[1],support,placement);
    return balance_control_apply_internal(s,target,input,enabled,standing,kp,kd,ki,support,placement);
}
#endif

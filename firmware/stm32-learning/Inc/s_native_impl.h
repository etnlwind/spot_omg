/* Include in exactly one translation unit so CAD mesh constants are shared
 * with the other gait implementations on the 320 KiB application slot. */
#include "s_native.h"
#include "arc_turn.h"
#include "s_native_data.h"
#include "s_native_v621_data.h"
#include "s_native_v623_data.h"
#include <string.h>
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "fp-contract=off")
#endif

static float smooth(float u) { return gait_policy_smootherstep(gait_policy_clampf(u,0,1)); }

void s_native_foot(int leg,const float q[3],float point[3],float jac[3][3]) {
    /* Same fixed sole material XY and lowest cushion Z as SoleKinematics. */
    arc_foot(leg,q,point,jac);
    float material[3];arc_transform(leg,q,3,sn_sole_zero[leg],material,false);
    point[0]=material[0];point[1]=material[1];
    if(!jac)return;
    for(int j=0;j<3;j++) {
        float origin[3],axis[3],v[3],cross[3];
        arc_transform(leg,q,j,arc_anchors[leg][j],origin,false);
        arc_transform(leg,q,j,arc_axes[leg][j],axis,true);
        for(int k=0;k<3;k++)v[k]=material[k]-origin[k];
        arc_cross(axis,v,cross);jac[0][j]=cross[0];jac[1][j]=cross[1];
    }
}

static bool solve(const float goal[4][3],float q[4][3],const float *locked) {
    const float low[3]={-30,-45,0},high[3]={30,100,150};
    if(locked)for(int i=0;i<4;i++) {
        if(!isfinite(locked[i]) || fabsf(locked[i])>30)return false;
        q[i][0]=locked[i];
    }
    for(int iteration=0;iteration<60;iteration++) {
        float increment[4][3]={{0}},maximum=0;
        for(int i=0;i<4;i++) {
            float point[3],jac[3][3],error[3],a[3][4]={{0}},dq[3]={0},norm=0;
            s_native_foot(i,q[i],point,jac);
            for(int r=0;r<3;r++) {
                error[r]=(locked && r==1)?0:goal[i][r]-point[r];norm+=error[r]*error[r];
            }
            maximum=fmaxf(maximum,norm);
            int first=locked?1:0;
            for(int j=first;j<3;j++) {
                for(int k=first;k<3;k++)for(int r=0;r<3;r++)
                    if(!locked || r!=1)a[j][k]+=jac[r][j]*jac[r][k];
                a[j][j]+=locked?1.e-9f:1.e-7f;
                for(int r=0;r<3;r++)a[j][3]+=jac[r][j]*error[r];
            }
            if(locked)a[0][0]=1;
            if(!arc_solve3(a,dq))return false;
            for(int j=first;j<3;j++)increment[i][j]=gait_policy_clampf(dq[j]*180/GAIT_POLICY_PI,-4,4);
        }
        if(maximum<(locked?1.e-10f:1.e-8f))break;
        for(int i=0;i<4;i++)for(int j=locked?1:0;j<3;j++)
            q[i][j]=gait_policy_clampf(q[i][j]+increment[i][j],low[j],high[j]);
    }
    for(int i=0;i<4;i++) {
        float point[3],error=0;s_native_foot(i,q[i],point,NULL);
        for(int j=0;j<3;j++)if(!locked || j!=1)error+=(point[j]-goal[i][j])*(point[j]-goal[i][j]);
        if(!isfinite(error) || error>(locked?4.e-8f:1.e-6f))return false;
    }
    return true;
}

void s_native_stand(GaitPolicyLegTarget out[4]) {
    for(int i=0;i<4;i++)out[i]=(GaitPolicyLegTarget){sn_standing[i][0],sn_standing[i][1],sn_standing[i][2],true};
}
void s_native_reset(SNativeControl *s) {
    memset(s,0,sizeof(*s));memcpy(s->previous,sn_standing,sizeof(s->previous));
    memcpy(s->normal,sn_standing,sizeof(s->normal));s->stop_progress=-1;
}
void s_native_reset_profile(SNativeControl *s,bool continuous_recovery) {
    s_native_reset(s);s->continuous_recovery=continuous_recovery;
}
void s_native_reset_v623(SNativeControl *s) {
    s_native_reset_profile(s,true);s->extended_reach=true;s->extended_rear_m=.115f;
}
void s_native_reset_v624(SNativeControl *s) {
    s_native_reset_v623(s);s->extended_rear_m=.105f;
}
void s_native_reset_v625(SNativeControl *s) {
    s_native_reset_v624(s);s->placement_swing=true;
}
void s_native_reset_v626(SNativeControl *s) {
    s_native_reset_v624(s);s->uniform_recovery=true;
}
void s_native_reset_v627(SNativeControl *s) {
    s_native_reset_v626(s);s->early_fold_recovery=true;
}
bool s_native_stopped(const SNativeControl *s) { return s->stop_progress>=1; }

static void sn_support_update(SNativeControl *s,float linear,float yaw) {
    if(fabsf(linear)+fabsf(yaw)<.1f)return; /* STOP retains the last reference. */
    if(s->have_support && s->support_linear==linear && s->support_yaw==yaw)return;
    s->support_linear=linear;s->support_yaw=yaw;s->have_support=true;
    float x=gait_policy_clampf((linear+1)*10,0,20),y=gait_policy_clampf((yaw+.5f)*10,0,10);
    int a=(int)fminf(19,floorf(x)),b=(int)fminf(9,floorf(y));float u=x-a,v=y-b;
    if(s->continuous_recovery) {
        const int16_t (*table)[11][1+2*SN_V621_HARMONICS]=s->extended_reach?sn_v623_support_um:sn_v621_support_um;
        float coefficient[1+2*SN_V621_HARMONICS];
        for(int k=0;k<1+2*SN_V621_HARMONICS;k++)coefficient[k]=1.e-6f*(
            (1-u)*((1-v)*table[a][b][k]+v*table[a][b+1][k])+
            u*((1-v)*table[a+1][b][k]+v*table[a+1][b+1][k]));
        for(int j=0;j<64;j++)s->support[j]=coefficient[0];
        /* Reconstruct once per requested input change. Each harmonic needs
         * one sin/cos pair, then a bounded oscillator recurrence. */
        for(int h=1;h<=SN_V621_HARMONICS;h++) {
            float angle=2*GAIT_POLICY_PI*h/64,dc=cosf(angle),ds=sinf(angle),c=1,t=0;
            for(int j=0;j<64;j++) {
                s->support[j]+=coefficient[2*h-1]*c+coefficient[2*h]*t;
                float next=c*dc-t*ds;t=t*dc+c*ds;c=next;
            }
        }
        return;
    }
    for(int j=0;j<64;j++)s->support[j]=1.e-6f*(
        (1-u)*((1-v)*sn_support_um[a][b][j]+v*sn_support_um[a][b+1][j])+
        u*((1-v)*sn_support_um[a+1][b][j]+v*sn_support_um[a+1][b+1][j]));
}

static bool sn_stop_place(SNativeControl *s,float dt,GaitPolicyLegTarget out[4]) {
    s->stop_progress=fminf(1,s->stop_progress+dt/(s->continuous_recovery?1.6f:1.2f));
    float goal[4][3],q[4][3],locked[4],progress[4];
    memcpy(goal,s->stop_feet,sizeof(goal));memcpy(q,s->previous,sizeof(q));
    for(int i=0;i<4;i++) {
        bool first=((i==0 || i==3)==s->stop_first_fl_rr);
        float u=gait_policy_clampf(2*s->stop_progress-(first?0:1),0,1);
        progress[i]=u;
        /* Shared normalized lift/placement/lower times; each diagonal uses
         * 0.6s in V6.1 and 0.8s in V6.2.1, without a final widening stage. */
        float move=smooth((u-.2f)/.2f),up=smooth(u/.2f),down=smooth((u-.45f)/.25f);
        float apex=fmaxf(s->stop_feet[i][2],sn_origin[i][2]+.012f);
        goal[i][0]+=(sn_origin[i][0]-goal[i][0])*move;
        goal[i][2]=u<.45f?s->stop_feet[i][2]+(apex-s->stop_feet[i][2])*up:
            apex+(sn_origin[i][2]-apex)*down;
        locked[i]=s->stop_j1[i]+(sn_standing[i][0]-s->stop_j1[i])*move;
    }
    if(!solve(goal,q,locked))return false;
    for(int i=0;i<4;i++) {
        if(progress[i]<=0)memcpy(q[i],s->stop_pose[i],sizeof(q[i]));
        if(progress[i]>=.7f)memcpy(q[i],sn_standing[i],sizeof(q[i]));
        out[i]=(GaitPolicyLegTarget){q[i][0],q[i][1],q[i][2],progress[i]<=0 || progress[i]>=.7f};
    }
    memcpy(s->previous,q,sizeof(q));
    return true;
}

bool s_native_step(SNativeControl *s,float phase,float amplitude,float linear,float yaw,
                   float requested_linear,float requested_yaw,float dt,bool stopping,GaitPolicyLegTarget out[4]) {
    if(!s || !out || !isfinite(phase) || phase<0 || phase>=1 || !isfinite(amplitude) || amplitude<0 || amplitude>1 ||
       !isfinite(linear) || fabsf(linear)>1 || !isfinite(yaw) || fabsf(yaw)>.501f ||
       !isfinite(requested_linear) || fabsf(requested_linear)>1 || !isfinite(requested_yaw) || fabsf(requested_yaw)>.501f ||
       !isfinite(dt) || dt<=0 || dt>.06f)return false;
    sn_support_update(s,requested_linear,requested_yaw);
    if(stopping && s->stop_progress<0) {
        s->stop_progress=0;s->stop_first_fl_rr=s->have_phase && s->last_phase<.5f;
        memcpy(s->stop_pose,s->previous,sizeof(s->stop_pose));
        for(int i=0;i<4;i++) {
            s->stop_j1[i]=s->previous[i][0];
            s_native_foot(i,s->previous[i],s->stop_feet[i],NULL);
        }
    }
    if(stopping)return sn_stop_place(s,dt,out);
    if(!stopping) {
        if(amplitude<=1.e-8f)s->entry_phase=0;
        else if(s->have_phase)s->entry_phase+=fmodf(phase-s->last_phase+1,1);
        s->last_phase=phase;s->have_phase=true;
    }
    float activity=fminf(1,(fabsf(linear)+fabsf(yaw))/.15f);
    if(!stopping && (amplitude<=1.e-8f || activity<=1.e-8f)) {
        memcpy(s->previous,sn_standing,sizeof(s->previous));s_native_stand(out);return true;
    }
    float goal[4][3],locked[4],q[4][3];memcpy(q,s->previous,sizeof(q));
    float cbody=1-2*smooth(fmodf(phase,.5f)/.5f);
    float transfer=-fminf(.006f,.225f*.04f*fabsf(linear))*(1-cbody*cbody);
    float p=phase*64;int index=(int)p;
    float support=activity*gait_policy_clampf(s->support[index]+(p-index)*(s->support[(index+1)%64]-s->support[index]),-.015f,.015f);
    const float offsets[4]={.5f,0,0,.5f};
    for(int i=0;i<4;i++) {
        float lp=fmodf(phase+offsets[i],1),c=1-2*smooth(fmodf(lp,.5f)/.5f);
        if(lp>=.5f)c=-c;
        if(s->placement_swing && lp>=.5f)
            c=-1+2*smooth(((lp-.5f)/.5f-.2f)/.6f);
        float back=fmaxf(0,-c),swing=(lp-.5f)/.5f;
        float lift=swing>0 && swing<1?sqrtf(fmaxf(0,sinf(GAIT_POLICY_PI*swing))):0;
        if(s->continuous_recovery)lift=sqrtf(lift);
        if(s->extended_reach)lift=smooth(fminf(swing,1-swing)/.35f);
        if(s->placement_swing || s->uniform_recovery)lift=smooth(fminf(swing,1-swing)/.3f);
        float height=.012f*activity*lift;
        if(s->early_fold_recovery)
            height=.016f*activity*smooth(fminf(swing,1-swing)/.25f);
        float rear=.045f;
        if(s->continuous_recovery) { back=(1-c)/2;rear=s->extended_reach?s->extended_rear_m:.105f; }
        /* Right-positive protocol yaw; stance feet oppose body rotation. */
        float x=.02f*linear*c-rear*linear*back*back*back;
        if(s->uniform_recovery)x=linear*(-rear/2+(.02f+rear/2)*c);
        if(s->early_fold_recovery && lp>=.5f) {
            float u=gait_policy_clampf(swing,0,1),v=1-u;
            float progress=1-v*v*v*v*v*(1+5*u+15*u*u);
            x+=linear*.145f*(progress-smooth(u));
        }
        goal[i][0]=sn_origin[i][0]+amplitude*(x+.10f*yaw*sn_origin[i][1]*c+transfer);
        goal[i][1]=sn_origin[i][1]-amplitude*.10f*yaw*sn_origin[i][0]*c+activity*sn_lateral[i]+amplitude*support;
        goal[i][2]=sn_origin[i][2]+height;
    }
    {
        if(!solve(goal,s->normal,NULL))return false;
        float blend=smooth((s->entry_phase-.5f)/.5f);
        for(int i=0;i<4;i++) {
            float placement=s->entry_phase<=.5f?(i==1?2*smooth(s->entry_phase/.5f):0):(i==1?2-blend:blend);
            locked[i]=sn_standing[i][0]+placement*sn_adduction[i]+blend*(s->normal[i][0]-sn_standing[i][0]-sn_adduction[i]);
            if(s->early_fold_recovery)locked[i]=sn_standing[i][0]+placement*sn_adduction[i];
        }
    }
    /* Keep the baseline X path, phase and J1 lock; change only swing Z. */
    if(s->diagnostic_leg) {
        if((s->diagnostic_leg!=3 && s->diagnostic_leg!=4 && s->diagnostic_leg!=5) ||
           !isfinite(s->diagnostic_lift_extra_m) || s->diagnostic_lift_extra_m<0 ||
           s->diagnostic_lift_extra_m>.028001f)return false;
        for(unsigned i=0;i<4;i++) {
        if(s->diagnostic_leg!=5 && i+1!=s->diagnostic_leg)continue;
        float swing=(fmodf(phase+offsets[i],1)-.5f)/.5f;
        goal[i][2]+=s->diagnostic_lift_extra_m*activity*
            smooth(fminf(swing,1-swing)/(s->early_fold_recovery?.25f:.3f));
        }
    }
    for(int i=0;i<4;i++) {
        float u=gait_policy_clampf((fmodf(phase+offsets[i],1)-.5f)*2,0,1);
        goal[i][2]+=s->foot_lift_mm[i]*.001f*amplitude*activity*64*u*u*u*(1-u)*(1-u)*(1-u);
    }
    if(s->diagnostic_width_active) {
        if(!isfinite(s->diagnostic_width_m) || s->diagnostic_width_m<-.040001f || s->diagnostic_width_m>.020001f)return false;
        float entry=smooth(s->entry_phase/.5f)*activity;
        for(int i=0;i<4;i++) {
            /* Absolute per-foot Y from calibrated vertical S, not a delta
             * from the old adduction. Width replaces the FR-only entry lock. */
            goal[i][1]=sn_origin[i][1]+(i%2==0?1:-1)*s->diagnostic_width_m*entry;
        }
        if(!solve(goal,q,NULL))return false;
        if(s->diagnostic_fr_extra && s->diagnostic_width_m<0 && s->entry_phase<1) {
            /* Double the mechanical J1 change, not the foot Y distance.
             * First half-cycle: FR only. Second: blend to common spacing. */
            float blend=smooth((s->entry_phase-.5f)/.5f);
            for(int i=0;i<4;i++) {
                float factor=i==1?2-blend:blend;
                locked[i]=sn_standing[i][0]+factor*(q[i][0]-sn_standing[i][0]);
            }
            if(!solve(goal,q,locked))return false;
        }
    } else if(!solve(goal,q,locked))return false;
    bool width=false;for(int i=0;i<4;i++)width|=s->foot_width_mm[i]!=0;
    if(width && amplitude*activity>0) {
        for(int i=0;i<4;i++) {
            s_native_foot(i,q[i],goal[i],NULL);
            goal[i][1]+=(i%2==0?1.f:-1.f)*s->foot_width_mm[i]*.001f*amplitude*activity;
        }
        if(!solve(goal,q,NULL))return false;
    }
    memcpy(s->previous,q,sizeof(q));
    for(int i=0;i<4;i++)out[i]=(GaitPolicyLegTarget){q[i][0],q[i][1],q[i][2],fmodf(phase+offsets[i],1)<.5f};
    return true;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

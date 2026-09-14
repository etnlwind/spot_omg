/* Isolated experimental binding. No plant state is read by this controller. */
#include "arc_turn.h"
#include "arc_dynamic_balance.h"

/* state=[dx,dy,vx,vy], config=[gain_scale,max_speed,max_accel,transition_seconds].
 * Angles/rates are radians and radians/sec. phase already includes target lead.
 * diagnostic=[wanted_ax,wanted_ay,applied_ax,applied_ay,pair0_weight,constrained].
 * Residual per-component position<=2.8mm implies vector norm<4mm. The caller must
 * separately ensure feedforward+this residual respects its total body envelope.
 */
int arc_dynamic_xy(float state[4],const float values[12],float phase,float period,
        const float imu[4],int available,float dt,const float config[4],float result[12],float diagnostic[6]) {
    if(!state||!values||!imu||!config||!result||!diagnostic||!isfinite(phase)||
       !isfinite(period)||period<.5f||period>2.f||!isfinite(dt)||dt<=0||dt>.06f)return 0;
    if(!isfinite(config[0])||config[0]<0||config[0]>2||
       !isfinite(config[1])||config[1]<=0||config[1]>.1f||
       !isfinite(config[2])||config[2]<=0||config[2]>1.f||
       !isfinite(config[3])||config[3]<=0||config[3]>.25f*period)return 0;
    bool feedback_active=available!=0&&config[0]>0;
    for(int i=0;i<4;i++)if(!isfinite(state[i])||(feedback_active&&!isfinite(imu[i])))return 0;
    GaitPolicyLegTarget pose[4];float feet[4][3];
    for(int i=0;i<4;i++){
        const float *q=values+i*3;
        if(!isfinite(q[0])||!isfinite(q[1])||!isfinite(q[2])||q[0]< -30||q[0]>30||q[1]< -45||q[1]>100||q[2]<0||q[2]>150)return 0;
        pose[i]=(GaitPolicyLegTarget){q[0],q[1],q[2],true};
        float point[3];arc_foot_geometry(i,q,point,NULL,feet[i]);
    }
    const int pairs[2][2]={{0,3},{1,2}};
    float weight=arc_dynamic_pair_weight(phase,period,config[3]),wanted[2]={0};
    for(int pair=0;pair<2;pair++){
        int a=pairs[pair][0],b=pairs[pair][1];
        float tx=feet[b][0]-feet[a][0],ty=feet[b][1]-feet[a][1];
        float length=sqrtf(tx*tx+ty*ty);if(length<1.e-5f)return 0;
        tx/=length;ty/=length;float nx=-ty,ny=tx;
        float theta=feedback_active?tx*imu[0]+ty*imu[1]:0;
        float rate=feedback_active?tx*imu[2]+ty*imu[3]:0;
        ArcDynamicAxisState projected={nx*state[0]+ny*state[1],nx*state[2]+ny*state[3]};
        float normal_acceleration;
        if(!arc_dynamic_acceleration(pair,theta,rate,&projected,feedback_active,&normal_acceleration))return 0;
        /* The direction along the active support line has no free-axis tilt
         * task. Damp its carried-over residual instead of leaving it to drift. */
        float tangent_acceleration=-20.f*(tx*state[0]+ty*state[1])-8.f*(tx*state[2]+ty*state[3]);
        float w=pair==0?weight:1-weight;
        wanted[0]+=w*(nx*normal_acceleration+tx*tangent_acceleration)*config[0];
        wanted[1]+=w*(ny*normal_acceleration+ty*tangent_acceleration)*config[0];
    }
    if(!feedback_active){
        /* LQR's state gain is not a safe standalone discrete translation PD:
         * Kv~100 at 20ms is near an alternating pole when theta feedback is
         * absent. Use the bounded, slower return law for sensor loss/off. */
        for(int k=0;k<2;k++)wanted[k]=-20.f*state[k]-8.f*state[k+2];
    }
    ArcDynamicAxisState next[2]={{state[0],state[2]},{state[1],state[3]}};float applied[2];
    for(int k=0;k<2;k++)if(!arc_dynamic_integrate_acceleration(&next[k],wanted[k],dt,.0028f,
        .707f*config[1],.707f*config[2],&applied[k]))return 0;
    for(int i=0;i<4;i++){
        float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg},point[3];arc_foot(i,q,point,NULL);
        point[0]-=next[0].delta;point[1]-=next[1].delta;
        if((next[0].delta!=0||next[1].delta!=0)&&!arc_ik(i,point,q))return 0;
        pose[i].j1_deg=q[0];pose[i].j2_deg=q[1];pose[i].j3_deg=q[2];
    }
    state[0]=next[0].delta;state[1]=next[1].delta;state[2]=next[0].velocity;state[3]=next[1].velocity;
    for(int i=0;i<4;i++){result[3*i]=pose[i].j1_deg;result[3*i+1]=pose[i].j2_deg;result[3*i+2]=pose[i].j3_deg;}
    for(int k=0;k<2;k++){diagnostic[k]=wanted[k];diagnostic[k+2]=applied[k];}
    diagnostic[4]=weight;diagnostic[5]=(fabsf(wanted[0]-applied[0])+fabsf(wanted[1]-applied[1]))>1.e-6f;
    return 1;
}

void arc_dynamic_xy_geometry(const float values[12],float centers[12],float contacts[12]) {
    for(int i=0;i<4;i++)arc_foot_geometry(i,values+3*i,centers+3*i,NULL,contacts+3*i);
}

#ifndef ARC_TRIPOD_SUPPORT_H
#define ARC_TRIPOD_SUPPORT_H
#include "arc_preload.h"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "fp-contract=off")
#endif

/* Minimal COM translation inside the edge opposite the scheduled swing foot.
 * Fixed CAD contact points only: no measured contact, force or true body state.
 * Positive body translation means the same translation is SUBTRACTED from all
 * body-frame foot XY targets. Not a centroid target or a stability guarantee. */
static inline bool arc_tripod_edge_shift(const float feet[4][3],const float com[3],
        int missing,float margin,float shift[2]) {
    if(missing<0||missing>3||!isfinite(margin)||margin<0||margin>.01f)return false;
    for(int k=0;k<3;k++)if(!isfinite(com[k]))return false;
    for(int i=0;i<4;i++)for(int k=0;k<3;k++)if(!isfinite(feet[i][k]))return false;
    int a=missing^1,b=missing^2,opposite=missing^3;
    float dx=feet[b][0]-feet[a][0],dy=feet[b][1]-feet[a][1],length=sqrtf(dx*dx+dy*dy);
    if(length<1.e-5f)return false;
    float nx=-dy/length,ny=dx/length;
    if(nx*(feet[opposite][0]-feet[a][0])+ny*(feet[opposite][1]-feet[a][1])<0){nx=-nx;ny=-ny;}
    float rx=com[0]-feet[a][0],ry=com[1]-feet[a][1];
    float along=(rx*dx+ry*dy)/(length*length),distance=rx*nx+ry*ny;
    if(distance>=margin&&along>=0&&along<=1){shift[0]=shift[1]=0;return true;}
    along=gait_policy_clampf(along,0,1);
    shift[0]=feet[a][0]+along*dx+margin*nx-com[0];
    shift[1]=feet[a][1]+along*dy+margin*ny-com[1];
    return true;
}

/* phase is the phase used by the target generator, including its lead. The
 * [0,.5,.75,.25] offsets yield RL,FR,RR,FL lift order. With duty>.75 a four-foot
 * interval appears before each next lift; all phase durations remain seconds
 * through the actual caller-supplied period. */
static inline bool arc_tripod_shift_goal(const float feet[4][3],const float com[3],
        float phase,float period,float duty,float margin,float transition,float shift[2]) {
    if(!isfinite(phase)||!isfinite(period)||period<.2f||period>10||!isfinite(duty)||duty<.75f||duty>.9f||
       !isfinite(transition)||transition<=0||transition>.25f*period)return false;
    float u=phase-(duty-.75f);u-=floorf(u);
    int quarter=(int)(u*4);if(quarter>3)quarter=3;
    const int order[4]={2,1,3,0};float now[2],next[2];
    if(!arc_tripod_edge_shift(feet,com,order[quarter],margin,now)||
       !arc_tripod_edge_shift(feet,com,order[(quarter+1)%4],margin,next))return false;
    float width=transition/period,local=u-quarter*.25f;
    float w=gait_policy_smootherstep(gait_policy_clampf((local-(.25f-width))/width,0,1));
    for(int k=0;k<2;k++)shift[k]=now[k]+w*(next[k]-now[k]);
    float norm=sqrtf(shift[0]*shift[0]+shift[1]*shift[1]);
    if(norm>.01f)for(int k=0;k<2;k++)shift[k]*=.01f/norm;
    return true;
}

static inline bool arc_tripod_support(float state[2],GaitPolicyLegTarget pose[4],
        float phase,float period,float duty,float margin,float transition) {
    if(!state||!pose||!isfinite(state[0])||!isfinite(state[1])||
       state[0]*state[0]+state[1]*state[1]>.00010001f)return false;
    for(int i=0;i<4;i++)if(!isfinite(pose[i].j1_deg)||!isfinite(pose[i].j2_deg)||!isfinite(pose[i].j3_deg)||
        pose[i].j1_deg< -30||pose[i].j1_deg>30||pose[i].j2_deg< -45||pose[i].j2_deg>100||pose[i].j3_deg<0||pose[i].j3_deg>150)return false;
    float bias[4][3],feet[4][3],jz[4][3],com[3],goal[2];arc_gravity_model(pose,bias,feet,jz,com);
    for(int i=0;i<4;i++){
        float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg},point[3];
        arc_foot_geometry(i,q,point,NULL,feet[i]);
    }
    if(!arc_tripod_shift_goal(feet,com,phase,period,duty,margin,transition,goal))return false;
    float delta[2]={goal[0]-state[0],goal[1]-state[1]},norm=sqrtf(delta[0]*delta[0]+delta[1]*delta[1]);
    if(norm>.001f)for(int k=0;k<2;k++)delta[k]*=.001f/norm;
    float next[2]={state[0]+delta[0],state[1]+delta[1]};GaitPolicyLegTarget out[4];
    for(int i=0;i<4;i++){
        float q[3]={pose[i].j1_deg,pose[i].j2_deg,pose[i].j3_deg},point[3];arc_foot(i,q,point,NULL);
        for(int k=0;k<2;k++)point[k]-=next[k];
        if(!arc_ik(i,point,q))return false;
        out[i]=pose[i];out[i].j1_deg=q[0];out[i].j2_deg=q[1];out[i].j3_deg=q[2];
    }
    for(int k=0;k<2;k++)state[k]=next[k];for(int i=0;i<4;i++)pose[i]=out[i];
    return true;
}

/* Optional static vertical force fractions. A COM outside the triangle is
 * rejected, never hidden by clipping negative forces. Pass the COM and contact
 * points from the SAME shifted pose. Does not imply friction/wrench feasibility. */
static inline bool arc_tripod_barycentric(const float feet[4][3],const float com[3],int missing,float share[4]) {
    if(missing<0||missing>3)return false;
    int a=missing^1,b=missing^2,c=missing^3;
    float bx=feet[b][0]-feet[a][0],by=feet[b][1]-feet[a][1];
    float cx=feet[c][0]-feet[a][0],cy=feet[c][1]-feet[a][1];
    float px=com[0]-feet[a][0],py=com[1]-feet[a][1],det=bx*cy-by*cx;
    if(!isfinite(det)||fabsf(det)<1.e-8f||!isfinite(px)||!isfinite(py))return false;
    float wb=(px*cy-py*cx)/det,wc=(bx*py-by*px)/det,wa=1-wb-wc;
    if(!isfinite(wa)||!isfinite(wb)||!isfinite(wc)||wa<0||wb<0||wc<0)return false;
    float result[4]={0};result[a]=wa;result[b]=wb;result[c]=wc;
    for(int i=0;i<4;i++)share[i]=result[i];return true;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

#ifndef ARC_TURN_H
#define ARC_TURN_H
#include "gait_policy.h"
#include "arc_geometry.h"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "fp-contract=off")
#endif
/* CAD space-screw kinematics. Canonical angles only; servo calibration remains
 * in locomotion_servo. Lowest cushion vertex supplies ground clearance. */
static inline void arc_cross(const float a[3],const float b[3],float o[3]) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
 for(int k=0;k<3;k++)o[k]=a[(k+1)%3]*b[(k+2)%3]-a[(k+2)%3]*b[(k+1)%3];
}
static inline void arc_rotate(const float axis[3],float angle,const float p[3],float o[3]) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
 float c=cosf(angle),s=sinf(angle),dot=0,cross[3];arc_cross(axis,p,cross);
 for(int k=0;k<3;k++)dot+=axis[k]*p[k];
 for(int k=0;k<3;k++)o[k]=p[k]*c+cross[k]*s+axis[k]*dot*(1-c);
}
static inline void arc_transform(int leg,const float q[3],int count,const float p[3],float o[3],bool direction) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
 for(int k=0;k<3;k++)o[k]=p[k];
 for(int j=count-1;j>=0;j--){
  float v[3],r[3];for(int k=0;k<3;k++)v[k]=o[k]-(direction?0:arc_anchors[leg][j][k]);
  arc_rotate(arc_axes[leg][j],q[j]*GAIT_POLICY_PI/180.f,v,r);
  for(int k=0;k<3;k++)o[k]=r[k]+(direction?0:arc_anchors[leg][j][k]);
 }
}
static inline float arc_projection(unsigned vertex,const float direction[3]) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
 return direction[0]*arc_vertices[vertex][0]+direction[1]*arc_vertices[vertex][1]+direction[2]*arc_vertices[vertex][2];
}
static inline float arc_bound(unsigned node,const float d[3]) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
 float value=0;
 for(int k=0;k<3;k++)value+=d[k]*arc_bounds[node][k+(d[k]<0?3:0)];
 return value;
}
static inline unsigned arc_lowest(const float direction[3]) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
 /* Depth-first branch-and-bound. A balanced 2000-point tree has depth 8;
  * use node-count capacity defensively. Bounds prune only with float margin. */
 uint16_t stack[ARC_NODE_COUNT];unsigned size=1,best=0;stack[0]=0;
 float value=arc_projection(best,direction);
 while(size){
  unsigned node=stack[--size];
  if(arc_bound(node,direction)>value+1.e-8f)continue;
  const uint16_t *entry=arc_nodes[node];
  if(entry[3]){
   for(unsigned i=entry[2];i<entry[2]+entry[3];i++){
    unsigned vertex=arc_order[i];float candidate=arc_projection(vertex,direction);
    if(candidate<value || (candidate==value && vertex<best)){value=candidate;best=vertex;}
   }
  }else{
   unsigned first=entry[0],second=entry[1];
   if(arc_bound(first,direction)>arc_bound(second,direction)){unsigned t=first;first=second;second=t;}
   stack[size++]=(uint16_t)second;stack[size++]=(uint16_t)first;
  }
 }
 return best;
}
static inline void arc_foot(int leg,const float q[3],float point[3],float jac[3][3]) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
 /* Reuse each joint's transform for center, cushion and Jacobian. The old
  * point-by-point composition recomputed trigonometry for every task. */
 float rotation[4][3][3]={{{1,0,0},{0,1,0},{0,0,1}}},translation[4][3]={{0}};
 for(int j=0;j<3;j++){
  float angle=q[j]*GAIT_POLICY_PI/180.f,c=cosf(angle),v=1-c,sn=sinf(angle);
  float x=arc_axes[leg][j][0],y=arc_axes[leg][j][1],z=arc_axes[leg][j][2];
  float r[3][3]={{c+x*x*v,x*y*v-z*sn,x*z*v+y*sn},{y*x*v+z*sn,c+y*y*v,y*z*v-x*sn},{z*x*v-y*sn,z*y*v+x*sn,c+z*z*v}};
  float t[3];
  for(int k=0;k<3;k++){
   t[k]=arc_anchors[leg][j][k];for(int n=0;n<3;n++)t[k]-=r[k][n]*arc_anchors[leg][j][n];
  }
  for(int k=0;k<3;k++){
   translation[j+1][k]=translation[j][k];
   for(int n=0;n<3;n++)translation[j+1][k]+=rotation[j][k][n]*t[n];
   for(int l=0;l<3;l++)for(int n=0;n<3;n++)rotation[j+1][k][l]+=rotation[j][k][n]*r[n][l];
  }
 }
 float orientation[3][3]={{0}},center[3],bottom[3];
 for(int k=0;k<3;k++){
  center[k]=translation[3][k];
  for(int n=0;n<3;n++)center[k]+=rotation[3][k][n]*arc_centers[leg][n];
  for(int l=0;l<3;l++)for(int n=0;n<3;n++)orientation[k][l]+=rotation[3][k][n]*arc_orientations[leg][n][l];
 }
 unsigned selected=arc_lowest(orientation[2]);
 for(int k=0;k<3;k++){
  bottom[k]=center[k];for(int j=0;j<3;j++)bottom[k]+=orientation[k][j]*arc_vertices[selected][j];
  point[k]=k==2?bottom[k]:center[k];
 }
 if(!jac)return;
 for(int j=0;j<3;j++){
  float origin[3],axis[3]={0},v[3],a[3],b[3];
  for(int k=0;k<3;k++){
   origin[k]=translation[j][k];
   for(int n=0;n<3;n++){
    origin[k]+=rotation[j][k][n]*arc_anchors[leg][j][n];
    axis[k]+=rotation[j][k][n]*arc_axes[leg][j][n];
   }
   v[k]=center[k]-origin[k];
  }
  arc_cross(axis,v,a);
  for(int k=0;k<3;k++)v[k]=bottom[k]-origin[k];
  arc_cross(axis,v,b);
  for(int k=0;k<3;k++)jac[k][j]=k==2?b[k]:a[k];
 }
}
static inline bool arc_solve3(float a[3][4],float out[3]) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
 for(int i=0;i<3;i++){
  int pivot=i;for(int j=i+1;j<3;j++)if(fabsf(a[j][i])>fabsf(a[pivot][i]))pivot=j;
  if(fabsf(a[pivot][i])<1.e-12f)return false;
  if(pivot!=i)for(int k=i;k<4;k++){float t=a[i][k];a[i][k]=a[pivot][k];a[pivot][k]=t;}
  float scale=a[i][i];for(int k=i;k<4;k++)a[i][k]/=scale;
  for(int j=0;j<3;j++)if(j!=i){float v=a[j][i];for(int k=i;k<4;k++)a[j][k]-=v*a[i][k];}
 }
 for(int i=0;i<3;i++)out[i]=a[i][3];
 return true;
}
static inline bool arc_ik(int leg,const float target[3],float q[3]) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
 static const float low[3]={-30,-45,0},high[3]={30,100,150};
 for(int iteration=0;iteration<16;iteration++){
  float point[3],jac[3][3],a[3][4]={{0}},dq[3],error[3],distance=0;
  arc_foot(leg,q,point,jac);
  for(int k=0;k<3;k++){error[k]=target[k]-point[k];distance+=error[k]*error[k];}
  if(distance<1.e-8f)return true;
  for(int j=0;j<3;j++){
   for(int k=0;k<3;k++)for(int r=0;r<3;r++)a[j][k]+=jac[r][j]*jac[r][k];
   a[j][j]+=1.e-7f;for(int r=0;r<3;r++)a[j][3]+=jac[r][j]*error[r];
  }
  if(!arc_solve3(a,dq))return false;
  for(int j=0;j<3;j++)q[j]=gait_policy_clampf(q[j]+gait_policy_clampf(dq[j]*180.f/GAIT_POLICY_PI,-4,4),low[j],high[j]);
 }
 float point[3],error=0;arc_foot(leg,q,point,NULL);
 for(int k=0;k<3;k++)error+=(point[k]-target[k])*(point[k]-target[k]);
 return error<2.5e-7f;
}
static inline void arc_seed(int leg,const float target[3],float q[3]) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
#ifndef ARC_HAS_SEEDS
 /* Geometry exporter bootstrap when replacing an older generated header. */
 for(int j=0;j<3;j++)q[j]=arc_neutral[leg*3+j];
#else
 /* Trilinear offline IK initialization. Clamp only the seed lookup, never
  * the requested foot target. arc_ik must still converge within its limits. */
 const int count[3]={9,5,3};int index[3];float fraction[3];
 for(int k=0;k<3;k++){
  float u=gait_policy_clampf((target[k]-arc_reference[leg][k]-arc_seed_origin[k])/arc_seed_step[k],0,(float)(count[k]-1));
  index[k]=(int)u;if(index[k]>=count[k]-1)index[k]=count[k]-2;
  fraction[k]=u-index[k];q[k]=0;
 }
 for(int x=0;x<2;x++)for(int y=0;y<2;y++)for(int z=0;z<2;z++){
  float weight=(x?fraction[0]:1-fraction[0])*(y?fraction[1]:1-fraction[1])*(z?fraction[2]:1-fraction[2]);
  for(int j=0;j<3;j++)q[j]+=weight*arc_seeds[leg][index[0]+x][index[1]+y][index[2]+z][j];
 }
#endif
}
static inline bool arc_turn_targets(float phase,float scale,float linear,float yaw,GaitPolicyLegTarget out[4]) {
#if defined(__clang__)
 #pragma STDC FP_CONTRACT OFF
#endif
 if(!isfinite(phase)||!isfinite(scale)||!isfinite(linear)||!isfinite(yaw)||scale<0||scale>1||fabsf(linear)>1||fabsf(yaw)>1)return false;
 float fraction=fabsf(yaw)/fmaxf(fabsf(linear)+fabsf(yaw),1.e-9f);
 float period=1.44f-.24f*fraction;
 phase+=.04f/period;
 float lift=fmaxf(.02f*fminf(1,fabsf(linear)+fabsf(yaw)),.02f*gait_policy_smootherstep(fminf(1,fabsf(yaw)/.5f)));
 for(int i=0;i<4;i++){
  float phase_i=phase+((i==1||i==2)?.5f:0);phase_i-=floorf(phase_i);
  float x,z=0;
  if(phase_i<.5f)x=.5f-2*phase_i;
  else{float u=2*phase_i-1;x=-.5f+2*gait_policy_smootherstep(u)-u;z=64*u*u*u*(1-u)*(1-u)*(1-u);}
  float angle=-scale*yaw*.2f*x,c=cosf(angle),s=sinf(angle);
  float dx=arc_reference[i][0]-arc_center[0],dy=arc_reference[i][1]-arc_center[1];
  float target[3]={arc_center[0]+c*dx-s*dy+scale*linear*.08f*x,arc_center[1]+s*dx+c*dy,arc_reference[i][2]+scale*lift*z};
  float q[3];arc_seed(i,target,q);
  if(!arc_ik(i,target,q))return false;
  out[i]=(GaitPolicyLegTarget){q[0],q[1],q[2],phase_i<.5f};
 }
 return true;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

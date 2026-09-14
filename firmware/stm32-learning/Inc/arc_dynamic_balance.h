#ifndef ARC_DYNAMIC_BALANCE_H
#define ARC_DYNAMIC_BALANCE_H
#include <math.h>
#include <stdbool.h>
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O2", "fp-contract=off")
#endif

/* Experimental fixed-neutral-CAD linear model, NOT torque control or a state
 * observer. x=[theta(rad),rate(rad/s),body translation(m),velocity(m/s)].
 * Ip*theta_ddot=m*g*h*theta-m*g*delta+m*h*delta_ddot.
 * The positive acceleration reaction is intentional. Offline continuous CARE:
 * Q=diag(1,.01,100,1), R=1. Existing IMU/safety checks remain independent.
 * Plant delay/saturation/contact changes are NOT covered by LQR pole stability.
 */
static const float arc_dynamic_mass=4.418f;
static const float arc_dynamic_height=.198298803f;
static const float arc_dynamic_inertia[2]={.227410649f,.227380853f};
static const float arc_dynamic_gain[2][4]={
    {-132.279431f,-21.5170432f,655.871665f,100.030531f},
    {-132.347277f,-21.5266693f,656.213806f,100.079297f}};
typedef struct {float delta,velocity;} ArcDynamicAxisState;

static inline float arc_dynamic_clip(float value,float low,float high){
    return fminf(high,fmaxf(low,value));
}
static inline bool arc_dynamic_acceleration(int pair,float theta,float rate,
        const ArcDynamicAxisState *state,bool available,float *acceleration){
    if(pair<0||pair>1||!state||!acceleration||!isfinite(state->delta)||!isfinite(state->velocity)||
       (available&&(!isfinite(theta)||!isfinite(rate))))return false;
    const float *k=arc_dynamic_gain[pair];
    *acceleration=-(k[2]*state->delta+k[3]*state->velocity);
    if(available)*acceleration-=k[0]*theta+k[1]*rate;
    return isfinite(*acceleration);
}

/* Constant-acceleration double-integrator step. The stopping-distance envelope
 * prevents clamping delta/velocity by teleporting them at the position limit.
 * Returns false for an already physically unbrakeable input state. Limits are
 * scalar-axis limits; blending two such axes needs separate 2D rate checking. */
static inline bool arc_dynamic_integrate_acceleration(ArcDynamicAxisState *state,
        float wanted,float dt,float max_delta,float max_velocity,
        float max_acceleration,float *applied_acceleration){
    if(!state||!applied_acceleration||!isfinite(dt)||dt<=0||dt>.06f||
       !isfinite(max_delta)||max_delta<=0||max_delta>.01f||
       !isfinite(max_velocity)||max_velocity<=0||max_velocity>.1f||
       !isfinite(max_acceleration)||max_acceleration<=0||max_acceleration>1.f)return false;
    if(!isfinite(wanted)||!isfinite(state->delta)||!isfinite(state->velocity))return false;
    if(fabsf(state->delta)>max_delta+1.e-7f||fabsf(state->velocity)>max_velocity+1.e-7f)return false;
    float stopping=state->delta+copysignf(state->velocity*state->velocity/(2*max_acceleration),state->velocity);
    if(fabsf(stopping)>max_delta+1.e-7f)return false;
    float low=fmaxf(-max_acceleration,(-max_velocity-state->velocity)/dt);
    float high=fminf(max_acceleration,(max_velocity-state->velocity)/dt);
    float acceleration=arc_dynamic_clip(wanted,low,high);
    /* Check both the path within this frame and the future stopping position.
     * If velocity reverses within the frame, endpoint-only checking misses the
     * turning point and could cross a limit before returning inside it. */
    for(int iteration=0;iteration<24;iteration++){
        float next_v=state->velocity+dt*acceleration;
        float next_d=state->delta+dt*state->velocity+.5f*dt*dt*acceleration;
        float end=next_d+copysignf(next_v*next_v/(2*max_acceleration),next_v);
        float minimum=fminf(state->delta,fminf(next_d,end)),maximum=fmaxf(state->delta,fmaxf(next_d,end));
        if(fabsf(acceleration)>1.e-12f){
            float turning_time=-state->velocity/acceleration;
            if(turning_time>0&&turning_time<dt){
                float turning_position=state->delta+turning_time*state->velocity+.5f*turning_time*turning_time*acceleration;
                minimum=fminf(minimum,turning_position);maximum=fmaxf(maximum,turning_position);
            }
        }
        if(maximum>max_delta){high=acceleration;acceleration=.5f*(low+high);}
        else if(minimum< -max_delta){low=acceleration;acceleration=.5f*(low+high);}
        else break;
    }
    float next_v=state->velocity+dt*acceleration;
    float next_d=state->delta+dt*state->velocity+.5f*dt*dt*acceleration;
    if(fabsf(next_d)>max_delta+1.e-7f||fabsf(next_v)>max_velocity+1.e-7f)return false;
    state->delta=next_d;state->velocity=next_v;*applied_acceleration=acceleration;
    return true;
}

static inline bool arc_dynamic_axis_step(int pair,ArcDynamicAxisState *state,
        float theta,float rate,bool available,float dt,float max_delta,float max_velocity,
        float max_acceleration,float *applied_acceleration){
    float wanted;if(!arc_dynamic_acceleration(pair,theta,rate,state,available,&wanted))return false;
    return arc_dynamic_integrate_acceleration(state,wanted,dt,max_delta,max_velocity,max_acceleration,applied_acceleration);
}

/* C2 scheduled pair weight only; a blend of moving delta targets must also
 * include w_dot/w_ddot when enforcing body velocity/acceleration constraints. */
static inline float arc_dynamic_pair_weight(float phase,float period,float transition_s){
    float width=arc_dynamic_clip(transition_s/period,.001f,.25f);
    float u=phase+width*.5f;u-=floorf(u);float x;
    if(u<width)x=u/width;
    else if(u<.5f)return 1;
    else if(u<.5f+width)x=1-(u-.5f)/width;
    else return 0;
    return x*x*x*(x*(x*6-15)+10);
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif

#ifndef BODY_STABILIZER_H
#define BODY_STABILIZER_H
/* Position PD only: no HAL, servo ticks, contacts, torque control or MPC.
 * Body frame: +X forward, +Y left, +Z up. All input angles/rates are SI.
 * gx/gy MUST be delivered gyroscope channels, never Euler finite differences.
 */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#define BODY_STABILIZER_HARD_MAX_FOOT_M 0.005f

typedef struct {
    float roll, pitch, gx, gy;
    uint32_t timestamp_ms, sequence;
    bool valid, axis_verified;
} BodyImuState;

typedef struct {
    float target_roll_rad, target_pitch_rad;
    float kp_roll, kp_pitch, kd_roll_s, kd_pitch_s;
    float angle_alpha, gyro_alpha;
    float max_roll_rad, max_pitch_rad, max_foot_offset_m;
    float max_error_rad, max_gyro_rad_s;
    uint32_t timeout_ms;
    float slew_rad_s, stance_transition_s;
} BodyStabilizerConfig;

typedef enum {
    BODY_STABILIZER_ACTIVE=0,
    BODY_STABILIZER_OFF,
    BODY_STABILIZER_NO_SAMPLE,
    BODY_STABILIZER_INVALID_SAMPLE,
    BODY_STABILIZER_AXES_UNVERIFIED,
    BODY_STABILIZER_STALE,
    BODY_STABILIZER_REPLAYED_SAMPLE,
    BODY_STABILIZER_ANGLE_GUARD,
    BODY_STABILIZER_GYRO_GUARD,
    BODY_STABILIZER_INVALID_DT,
    BODY_STABILIZER_INVALID_GEOMETRY,
    BODY_STABILIZER_INVALID_CONFIG,
    BODY_STABILIZER_INVALID_STATE
} BodyStabilizerStatus;

typedef struct {
    BodyStabilizerStatus status;
    bool enabled, new_sample, input_finite, filter_initialized;
    uint32_t sample_age_ms, sample_sequence, sample_timestamp_ms;
    float raw[4], filtered[4]; /* roll, pitch, gx, gy */
    float error[2], u_raw[2], u_requested[2], u_applied[2];
    float stance_weights[4], requested_dz[4], applied_dz[4];
    uint8_t axis_clamp_mask, foot_clamp_mask;
    bool slew_limited;
} BodyStabilizerDiagnostics;

typedef struct {
    bool initialized;
    uint32_t sample_timestamp_ms, sample_sequence;
    float raw[4], filtered[4], u[2];
    BodyStabilizerDiagnostics diagnostics;
} BodyStabilizerState;

static inline float body_stabilizer_clip(float value,float limit) {
    return fmaxf(-limit,fminf(limit,value));
}
static inline float body_stabilizer_smooth(float value) {
    float x=fmaxf(0.f,fminf(1.f,value));
    /* Float roundoff near one can produce 1.0000006 and falsely trip the
     * stance-weight geometry contract, even with stabilization disabled. */
    return fmaxf(0.f,fminf(1.f,x*x*x*(10.f+x*(-15.f+6.f*x))));
}
static inline void body_stabilizer_reset(BodyStabilizerState *state) {
    if(state) {
        memset(state,0,sizeof(*state));
        state->diagnostics.status=BODY_STABILIZER_NO_SAMPLE;
    }
}
static inline const char *body_stabilizer_status_name(BodyStabilizerStatus status) {
    switch(status) {
    case BODY_STABILIZER_ACTIVE:return "active";
    case BODY_STABILIZER_OFF:return "off";
    case BODY_STABILIZER_NO_SAMPLE:return "no-sample";
    case BODY_STABILIZER_INVALID_SAMPLE:return "invalid-sample";
    case BODY_STABILIZER_AXES_UNVERIFIED:return "axes-unverified";
    case BODY_STABILIZER_STALE:return "stale";
    case BODY_STABILIZER_REPLAYED_SAMPLE:return "replayed-sample";
    case BODY_STABILIZER_ANGLE_GUARD:return "angle-guard";
    case BODY_STABILIZER_GYRO_GUARD:return "gyro-guard";
    case BODY_STABILIZER_INVALID_DT:return "invalid-dt";
    case BODY_STABILIZER_INVALID_GEOMETRY:return "invalid-geometry";
    case BODY_STABILIZER_INVALID_CONFIG:return "invalid-config";
    case BODY_STABILIZER_INVALID_STATE:return "invalid-state";
    }
    return "unknown";
}

static inline bool body_stabilizer_config_valid(const BodyStabilizerConfig *c) {
    if(!c)return false;
    return isfinite(c->target_roll_rad)&&fabsf(c->target_roll_rad)<=.35f&&
        isfinite(c->target_pitch_rad)&&fabsf(c->target_pitch_rad)<=.35f&&
        isfinite(c->kp_roll)&&c->kp_roll>=0&&c->kp_roll<=2.f&&
        isfinite(c->kp_pitch)&&c->kp_pitch>=0&&c->kp_pitch<=2.f&&
        isfinite(c->kd_roll_s)&&c->kd_roll_s>=0&&c->kd_roll_s<=.2f&&
        isfinite(c->kd_pitch_s)&&c->kd_pitch_s>=0&&c->kd_pitch_s<=.2f&&
        isfinite(c->angle_alpha)&&c->angle_alpha>=0&&c->angle_alpha<1.f&&
        isfinite(c->gyro_alpha)&&c->gyro_alpha>=0&&c->gyro_alpha<1.f&&
        isfinite(c->max_roll_rad)&&c->max_roll_rad>0&&c->max_roll_rad<=.2f&&
        isfinite(c->max_pitch_rad)&&c->max_pitch_rad>0&&c->max_pitch_rad<=.2f&&
        isfinite(c->max_foot_offset_m)&&c->max_foot_offset_m>0&&
        c->max_foot_offset_m<=BODY_STABILIZER_HARD_MAX_FOOT_M&&
        isfinite(c->max_error_rad)&&c->max_error_rad>0&&c->max_error_rad<=.6f&&
        isfinite(c->max_gyro_rad_s)&&c->max_gyro_rad_s>0&&c->max_gyro_rad_s<=10.f&&
        c->timeout_ms>0&&c->timeout_ms<=1000U&&
        isfinite(c->slew_rad_s)&&c->slew_rad_s>0&&c->slew_rad_s<=2.f&&
        isfinite(c->stance_transition_s)&&c->stance_transition_s>0&&c->stance_transition_s<=.25f;
}

/* Use the phase of the NOMINAL TARGET being corrected, not the phase after
 * the gait clock has advanced. Weights describe planned stance, not contacts.
 * The returned weight is exactly zero for the entire scheduled swing.
 */
static inline bool body_stabilizer_stance_weights_offsets(float phase,float duty,
        float period_s,float transition_s,const float offsets[4],float weights[4]) {
    if(!weights)return false;
    for(int i=0;i<4;i++)weights[i]=0.f;
    if(!isfinite(phase)||!isfinite(duty)||duty<.5f||duty>=1.f||
       !isfinite(period_s)||period_s<=0||!isfinite(transition_s)||transition_s<=0)return false;
    float edge=fminf(transition_s/period_s,.5f*duty);
    if(!isfinite(edge)||edge<=0)return false;
    for(int i=0;i<4;i++) {
        float q=phase+offsets[i];q-=floorf(q);
        if(q<duty)weights[i]=body_stabilizer_smooth(q/edge)*
                                  body_stabilizer_smooth((duty-q)/edge);
    }
    return true;
}
static inline bool body_stabilizer_stance_weights(float phase,float duty,
        float period_s,float transition_s,float weights[4]) {
    const float offsets[4]={0,.5f,.5f,0};
    return body_stabilizer_stance_weights_offsets(phase,duty,period_s,transition_s,offsets,weights);
}

/* On a mathematical input-contract error, return false and a finite bounded
 * previous correction (zero where a valid new weight says swing). The caller
 * must retain its last valid full motor command / report the contract error;
 * false is NOT an instruction to disable motor torque. Unknown geometry or dt
 * cannot support a smooth fresh trajectory. Normal OFF/stale/guard handling
 * instead returns true and fades latent u using the valid control dt.
 */
static inline bool body_stabilizer_contract_error(BodyStabilizerState *s,
        BodyStabilizerStatus status,const float weights[4],float out[4][3]) {
    if(out)for(int i=0;i<4;i++) {
        out[i][0]=out[i][1]=0.f;
        float previous=s?s->diagnostics.applied_dz[i]:0.f;
        if(!isfinite(previous))previous=0.f;
        out[i][2]=body_stabilizer_clip(previous,BODY_STABILIZER_HARD_MAX_FOOT_M);
        if(weights&&isfinite(weights[i])&&weights[i]==0.f)out[i][2]=0.f;
    }
    if(s)s->diagnostics.status=status;
    return false;
}

static inline bool body_stabilizer_update(BodyStabilizerState *s,
        const BodyStabilizerConfig *c,const BodyImuState *sample,
        uint32_t now_ms,float dt,bool enabled,const float feet_body[4][3],
        const float weights[4],float out[4][3]) {
    if(!s||!out)return body_stabilizer_contract_error(s,BODY_STABILIZER_INVALID_STATE,weights,out);
    if(!body_stabilizer_config_valid(c))
        return body_stabilizer_contract_error(s,BODY_STABILIZER_INVALID_CONFIG,weights,out);
    if(!isfinite(dt)||dt<=0.f||dt>.1f)
        return body_stabilizer_contract_error(s,BODY_STABILIZER_INVALID_DT,weights,out);
    if(!feet_body||!weights)
        return body_stabilizer_contract_error(s,BODY_STABILIZER_INVALID_GEOMETRY,weights,out);
    for(int i=0;i<4;i++) {
        if(!isfinite(weights[i])||weights[i]<0.f||weights[i]>1.f)
            return body_stabilizer_contract_error(s,BODY_STABILIZER_INVALID_GEOMETRY,weights,out);
        for(int j=0;j<3;j++)if(!isfinite(feet_body[i][j])||fabsf(feet_body[i][j])>2.f)
            return body_stabilizer_contract_error(s,BODY_STABILIZER_INVALID_GEOMETRY,weights,out);
    }
    for(int j=0;j<2;j++)if(!isfinite(s->u[j])||fabsf(s->u[j])>.200001f)
        return body_stabilizer_contract_error(s,BODY_STABILIZER_INVALID_STATE,weights,out);
    if(s->initialized)for(int j=0;j<4;j++)if(!isfinite(s->filtered[j])||!isfinite(s->raw[j]))
        return body_stabilizer_contract_error(s,BODY_STABILIZER_INVALID_STATE,weights,out);

    BodyStabilizerDiagnostics d={0};d.enabled=enabled;
    BodyStabilizerStatus sensor_status=BODY_STABILIZER_NO_SAMPLE;
    if(sample) {
        const float raw[4]={sample->roll,sample->pitch,sample->gx,sample->gy};
        d.input_finite=true;
        for(int j=0;j<4;j++){d.raw[j]=raw[j];d.input_finite=d.input_finite&&isfinite(raw[j]);}
        if(!sample->valid||!d.input_finite)sensor_status=BODY_STABILIZER_INVALID_SAMPLE;
        else if(!sample->axis_verified)sensor_status=BODY_STABILIZER_AXES_UNVERIFIED;
        else if((int32_t)(now_ms-sample->timestamp_ms)<0)sensor_status=BODY_STABILIZER_INVALID_SAMPLE;
        else if(s->initialized && sample->sequence!=s->sample_sequence &&
                ((int32_t)(sample->sequence-s->sample_sequence)<=0 ||
                 (int32_t)(sample->timestamp_ms-s->sample_timestamp_ms)<0))
            sensor_status=BODY_STABILIZER_REPLAYED_SAMPLE;
        else if(fabsf(c->target_roll_rad-raw[0])>c->max_error_rad ||
                fabsf(c->target_pitch_rad-raw[1])>c->max_error_rad)
            sensor_status=BODY_STABILIZER_ANGLE_GUARD;
        else if(fabsf(raw[2])>c->max_gyro_rad_s||fabsf(raw[3])>c->max_gyro_rad_s)
            sensor_status=BODY_STABILIZER_GYRO_GUARD;
        else {
            bool fresh=!s->initialized||sample->sequence!=s->sample_sequence;
            uint32_t age=now_ms-(fresh?sample->timestamp_ms:s->sample_timestamp_ms);
            sensor_status=age>c->timeout_ms?BODY_STABILIZER_STALE:BODY_STABILIZER_ACTIVE;
            if(fresh&&sensor_status==BODY_STABILIZER_ACTIVE) {
                for(int j=0;j<4;j++) {
                    float alpha=j<2?c->angle_alpha:c->gyro_alpha;
                    /* alpha is previous-value weight: 0 bypasses LPF; 1 is
                     * rejected because a frozen filter is not a sensor. */
                    s->filtered[j]=s->initialized?alpha*s->filtered[j]+(1.f-alpha)*raw[j]:raw[j];
                    s->raw[j]=raw[j];
                }
                s->sample_timestamp_ms=sample->timestamp_ms;
                s->sample_sequence=sample->sequence;s->initialized=true;d.new_sample=true;
            }
        }
    }
    d.status=enabled?sensor_status:BODY_STABILIZER_OFF;
    d.filter_initialized=s->initialized;
    d.sample_timestamp_ms=s->sample_timestamp_ms;d.sample_sequence=s->sample_sequence;
    d.sample_age_ms=s->initialized?now_ms-s->sample_timestamp_ms:UINT32_MAX;
    for(int j=0;j<4;j++)d.filtered[j]=s->filtered[j];
    if(s->initialized){d.error[0]=c->target_roll_rad-s->filtered[0];d.error[1]=c->target_pitch_rad-s->filtered[1];}
    if(enabled&&sensor_status==BODY_STABILIZER_ACTIVE) {
        d.u_raw[0]=c->kp_roll*d.error[0]-c->kd_roll_s*s->filtered[2];
        d.u_raw[1]=c->kp_pitch*d.error[1]-c->kd_pitch_s*s->filtered[3];
        for(int j=0;j<2;j++) {
            d.u_requested[j]=body_stabilizer_clip(d.u_raw[j],j==0?c->max_roll_rad:c->max_pitch_rad);
            if(d.u_requested[j]!=d.u_raw[j])d.axis_clamp_mask|=(uint8_t)(1U<<j);
        }
    }
    for(int j=0;j<2;j++) {
        float step=d.u_requested[j]-s->u[j],limit=c->slew_rad_s*dt;
        if(fabsf(step)>limit)d.slew_limited=true;
        s->u[j]+=body_stabilizer_clip(step,limit);d.u_applied[j]=s->u[j];
    }
    for(int i=0;i<4;i++) {
        d.stance_weights[i]=weights[i];
        d.requested_dz[i]=weights[i]*(-d.u_requested[0]*feet_body[i][1]+d.u_requested[1]*feet_body[i][0]);
        float dz=-s->u[0]*feet_body[i][1]+s->u[1]*feet_body[i][0];
        float bounded=body_stabilizer_clip(dz,c->max_foot_offset_m);
        if(dz!=bounded&&weights[i]>0)d.foot_clamp_mask|=(uint8_t)(1U<<i);
        /* Slew latent global u first, then stance weighting. A stale/off
         * residual therefore never leaks into a scheduled swing foot. */
        d.applied_dz[i]=weights[i]*bounded;
        out[i][0]=out[i][1]=0.f;out[i][2]=d.applied_dz[i];
    }
    s->diagnostics=d;
    return true;
}
#endif

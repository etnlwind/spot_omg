#include "servo_response_probe.h"
#include "sts3215.h"
#include <string.h>

enum { RR=11, PERIOD_MS=5, BASELINE_MS=200, STEP_MS=500 };
static unsigned mag(int n) { return (unsigned)(n<0?-n:n); }
static uint16_t word(const uint8_t *p) { return (uint16_t)(p[0]|((uint16_t)p[1]<<8)); }
static void put(uint8_t *p,uint16_t n) { p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8); }

static RobotResult attitude(RobotController *r,ServoResponseProbeReport *d) {
    int16_t roll=0,pitch=0;
    if(!r->attitude_reader || !r->attitude_reader(r->attitude_context,&roll,&pitch))return ROBOT_IMU_ERROR;
    roll-=ROBOT_IMU_LEVEL_ROLL_TENTHS;pitch-=ROBOT_IMU_LEVEL_PITCH_TENTHS;
    if(mag(roll)>mag(d->peak_roll_tenths))d->peak_roll_tenths=roll;
    if(mag(pitch)>mag(d->peak_pitch_tenths))d->peak_pitch_tenths=pitch;
    return mag(roll)>50 || mag(pitch)>50 ? ROBOT_TILT_LIMIT : ROBOT_OK;
}

static ServoBusResult read_sample(RobotController *r,unsigned j,Sts3215State *s,bool record) {
    memset(s,0,sizeof(*s));
    uint32_t begin=HAL_GetTick();
    ServoBusResult b=sts3215_read_state(r->bus,g_robot_servo_ids[j],s);
    if(record)joint_trace_sample(&r->joint_trace,(JointTraceSample){
        .begin_ms=begin,.end_ms=HAL_GetTick(),.joint=(uint8_t)j,.status=(uint8_t)b,
        .position=s->position,.speed=s->speed,.load=s->load,.current=s->current,
        .voltage_mv=s->voltage_mv,.temperature=s->temperature_c,.hardware_error=s->hardware_error});
    return b;
}

static RobotResult observe(RobotController *r,unsigned j,const uint16_t goals[12],
                          const uint16_t initial[12],ServoResponseProbeReport *d,
                          Sts3215State *s,bool record) {
    d->failed_id=g_robot_servo_ids[j];
    ServoBusResult b=read_sample(r,j,s,record);
    if(b!=SERVO_BUS_OK){r->last_bus_result=b;return ROBOT_BUS_ERROR;}
    if(s->temperature_c>=70 && !s->hardware_error && s->voltage_mv>=10500) {
        /* v74 observed 41 -> 70 -> 41 C in consecutive observations. Keep
         * the suspect record, pause sequencing and cross-check register 63
         * separately plus a second full state. Never raise the 70 C limit. */
        d->temperature_suspects++;HAL_Delay(15);
        uint8_t temperature=0;
        b=servo_bus_read(r->bus,g_robot_servo_ids[j],63,&temperature,1);
        if(b!=SERVO_BUS_OK){r->last_bus_result=b;return ROBOT_BUS_ERROR;}
        b=read_sample(r,j,s,record);
        if(b!=SERVO_BUS_OK){r->last_bus_result=b;return ROBOT_BUS_ERROR;}
        if(temperature>=70 || s->temperature_c>=70)return ROBOT_VERIFY_ERROR;
        d->temperature_recovered++;
    }
    if(s->hardware_error || s->temperature_c>=70 || s->voltage_mv<10500 ||
       mag(s->load)>=r->safety.limits.load_magnitude ||
       mag(s->current)>=r->safety.limits.current_magnitude)return ROBOT_VERIFY_ERROR;
    if(s->position>4095 || (j==RR && mag((int)s->position-goals[j])>64) ||
       (j!=RR && mag((int)s->position-initial[j])>24))return ROBOT_POSITION_LIMIT;
    d->failed_id=0;return ROBOT_OK;
}

static RobotResult capture(RobotController *r,uint32_t duration,const uint16_t goals[12],
                           const uint16_t initial[12],ServoResponseProbeReport *d) {
    uint32_t begin=HAL_GetTick(),next=begin,other_at=begin,last_change=begin;
    unsigned other=0;uint16_t previous=UINT16_MAX,anchor=UINT16_MAX;Sts3215State state;
    while(HAL_GetTick()-begin<duration) {
        if(r->motion_abort_requested || r->drive_stop_requested)return ROBOT_MOTION_ABORTED;
        RobotResult result=attitude(r,d);if(result!=ROBOT_OK)return result;
        result=observe(r,RR,goals,initial,d,&state,true);if(result!=ROBOT_OK)return result;
        if(anchor==UINT16_MAX || mag((int)state.position-anchor)>2) {
            last_change=HAL_GetTick();anchor=state.position;
        }
        previous=state.position;
        if(HAL_GetTick()-other_at>=20) {
            other_at=HAL_GetTick();
            result=observe(r,other,goals,initial,d,&state,true);if(result!=ROBOT_OK)return result;
            other=(other+1)%RR;
        }
        next+=PERIOD_MS;
        uint32_t now=HAL_GetTick();
        if((int32_t)(next-now)>0)HAL_Delay(next-now);else next=now;
    }
    /* A stable response is required before the reverse step, but static
     * load error is retained in the recorded data, not called exact arrival. */
    if(HAL_GetTick()-last_change<60 || mag((int)previous-goals[RR])>24)return ROBOT_VERIFY_ERROR;
    return ROBOT_OK;
}

RobotResult robot_probe_rr_j3(RobotController *r,uint8_t acceleration,
                             uint16_t delta_ticks,ServoResponseProbeReport *d) {
    if(!d)return ROBOT_INVALID_ARGUMENT;
    memset(d,0,sizeof(*d));
    if(!r || !r->bus || (acceleration!=10 && acceleration!=30 && acceleration!=50) ||
       delta_ticks<12 || delta_ticks>34 || r->drive_active || r->stow_active ||
       r->gait_diagnostics_active || r->motion_abort_requested || r->drive_stop_requested)
        return ROBOT_INVALID_ARGUMENT;
    if(safety_is_faulted(&r->safety) || r->locomotion_fault)return ROBOT_SAFETY_FAULT;
    uint16_t nominal[12],goals[12],initial[12];uint8_t reg[8];Sts3215State s;
    if(!robot_stand_targets(nominal))return ROBOT_CONFIG_ERROR;
    RobotResult result=attitude(r,d);if(result!=ROBOT_OK)return result;
    /* Read every live torque/goal without reconstructing front J2 commands
     * from modulo feedback. Only the RR J3 block will ever be written. */
    for(unsigned j=0;j<12;j++) {
        d->failed_id=g_robot_servo_ids[j];
        if(servo_bus_read(r->bus,g_robot_servo_ids[j],40,reg,8)!=SERVO_BUS_OK)return ROBOT_BUS_ERROR;
        if(reg[0]!=1)return ROBOT_VERIFY_ERROR;
        goals[j]=word(&reg[2]);
        if(j==RR)memcpy(d->original,reg+1,7);
        if(sts3215_read_state(r->bus,g_robot_servo_ids[j],&s)!=SERVO_BUS_OK)return ROBOT_BUS_ERROR;
        initial[j]=s.position;
        if(s.position>4095 || mag((int)s.position-nominal[j])>80)return ROBOT_STAND_REQUIRED;
        if(s.hardware_error || s.temperature_c>=70 || s.voltage_mv<10500 ||
           mag(s.load)>=r->safety.limits.load_magnitude || mag(s.current)>=r->safety.limits.current_magnitude)
            return ROBOT_VERIFY_ERROR;
        /* Preserve raw goals in trace; supporting-axis drift is measured
         * against initial positions, not against signed J2 goal words. */
    }
    if(goals[RR]>4095-delta_ticks || mag((int)goals[RR]-initial[RR])>16)return ROBOT_VERIFY_ERROR;
    d->failed_id=0;d->stage=1;
    r->shared_idle=false; /* Preserve support goals, no balance writes during identification. */
    joint_trace_arm(&r->joint_trace);
    r->joint_trace.profile=(uint8_t)r->locomotion_profile;
    r->joint_trace.speed=3400;r->joint_trace.acceleration=acceleration;
    uint16_t original_goal=goals[RR];uint8_t profile[7],readback[7];
    memcpy(profile,d->original,7);profile[0]=acceleration;put(profile+3,0);put(profile+5,3400);
    /* Same existing goal, only the experimental profile changes. */
    if(servo_bus_write(r->bus,12,41,profile,7)!=SERVO_BUS_OK){result=ROBOT_BUS_ERROR;goto failed;}
    if(servo_bus_read(r->bus,12,41,d->applied,7)!=SERVO_BUS_OK){result=ROBOT_BUS_ERROR;goto failed;}
    if(memcmp(profile,d->applied,7)){result=ROBOT_ACTUATOR_PROFILE_ERROR;goto failed;}
    uint32_t now=HAL_GetTick();joint_trace_command(&r->joint_trace,now,now,goals);
    result=capture(r,BASELINE_MS,goals,initial,d);if(result!=ROBOT_OK)goto failed;
    for(unsigned step=0;step<2;step++) {
        d->stage=(uint8_t)(2+step);
        if(r->motion_abort_requested || r->drive_stop_requested){result=ROBOT_MOTION_ABORTED;goto failed;}
        result=attitude(r,d);if(result!=ROBOT_OK)goto failed;
        goals[RR]=step==0?(uint16_t)(original_goal+delta_ticks):original_goal;
        put(profile+1,goals[RR]);now=HAL_GetTick();
        ServoBusResult b=servo_bus_write(r->bus,12,41,profile,7);
        joint_trace_command(&r->joint_trace,now,HAL_GetTick(),goals);
        if(b!=SERVO_BUS_OK){result=ROBOT_BUS_ERROR;goto failed;}
        result=capture(r,STEP_MS,goals,initial,d);if(result!=ROBOT_OK)goto failed;
    }
    d->stage=4;r->joint_trace.armed=false;
    if(servo_bus_write(r->bus,12,41,d->original,7)!=SERVO_BUS_OK){result=ROBOT_BUS_ERROR;goto failed;}
    if(servo_bus_read(r->bus,12,41,d->final,7)!=SERVO_BUS_OK){result=ROBOT_BUS_ERROR;goto failed;}
    if(memcmp(d->original,d->final,7)){result=ROBOT_VERIFY_ERROR;goto failed;}
    d->restored=true;return ROBOT_OK;

failed:
    r->joint_trace.armed=false;r->joint_trace.arm_on_stop=false;
    r->last_failed_servo_id=d->failed_id;
    /* No blind reverse or torque drop on a failed floor trial. RR J3 uses
     * ordinary single-turn coordinates; never perform this on a front J2. */
    if(sts3215_read_state(r->bus,12,&s)==SERVO_BUS_OK && s.position<=4095 &&
       mag((int)s.position-initial[RR])<=64) {
        memcpy(readback,d->original,7);put(readback+1,s.position);
        d->hold_sent=servo_bus_write(r->bus,12,41,readback,7)==SERVO_BUS_OK;
        (void)servo_bus_read(r->bus,12,41,d->final,7);
    }
    return result;
}

#include "mechanical_diagnostics.h"
#include "robot_config.h"
#include "sts3215.h"
#include "feetech_protocol.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static bool emit(MechanicalLogWriter writer, void *context, const char *format, ...)
{
    char line[96];
    va_list args;
    va_start(args,format);
    int n=vsnprintf(line,sizeof(line),format,args);
    va_end(args);
    return n>=0 && (size_t)n<sizeof(line) && writer(context,line);
}
static bool valid_label(const char *label)
{
    if(!label || !label[0])return false;
    size_t n=0;
    for(;label[n];n++) {
        char c=label[n];
        if(n>=16 || !((c>='a' && c<='z') || (c>='A' && c<='Z') ||
                     (c>='0' && c<='9') || c=='_' || c=='-'))return false;
    }
    return true;
}
static long angle10(int32_t ticks, int direction)
{
    return (long)((int64_t)ticks*direction*3600/4096);
}
static long mean10(int64_t sum, uint32_t count, int direction)
{
    return count ? (long)(sum*direction*3600/((int64_t)count*4096)) : 0;
}

bool mechanical_log_capture(ServoBus *bus, bool moving, const char *label,
                            MechanicalLogWriter writer, void *context)
{
    if(!bus || moving || !writer || !valid_label(label))return false;
    bool ok=emit(writer,context,"MECH_BEGIN mode=pose label=%s units=angle10,current_raw",label);
    unsigned valid=0,missing=0;
    for(size_t i=0;i<ROBOT_JOINT_COUNT;i++) {
        const RobotJointConfig *c=&g_robot_joints[i];
        Sts3215State state;
        ServoBusResult result=sts3215_read_state(bus,c->servo_id,&state);
        if(result!=SERVO_BUS_OK) {
            ok=emit(writer,context,"MECH_MISSING id=%u result=%u",c->servo_id,result) && ok;
            missing++;continue;
        }
        uint8_t raw[4];
        ServoBusResult goal_result=servo_bus_read(bus,c->servo_id,STS3215_ADDR_TORQUE_ENABLE,raw,sizeof(raw));
        valid++;
        ok=emit(writer,context,"MECH_POS id=%u leg=%u j=%u pos=%u angle10=%ld moving=%u",
                c->servo_id,c->leg_index,c->joint_index,state.position,
                angle10((int32_t)state.position-c->center,c->direction),state.moving) && ok;
        ok=emit(writer,context,"MECH_PWR id=%u speed=%d load=%d current_raw=%d mv=%u temp=%u hw=%u",
                c->servo_id,state.speed,state.load,state.current,state.voltage_mv,
                state.temperature_c,state.hardware_error) && ok;
        ok=emit(writer,context,"MECH_CAL id=%u ctr=%u dir=%d min=%u max=%u",
                c->servo_id,c->center,c->direction,c->minimum,c->maximum) && ok;
        if(goal_result==SERVO_BUS_OK) {
            uint16_t goal=feetech_decode_u16(raw+2);
            ok=emit(writer,context,"MECH_GOAL id=%u goal=%u err10=%ld torque=%u",
                    c->servo_id,goal,angle10((int32_t)goal-state.position,c->direction),raw[0]) && ok;
        } else {
            missing++;
            ok=emit(writer,context,"MECH_GOAL_MISSING id=%u result=%u",c->servo_id,goal_result) && ok;
        }
    }
    ok=emit(writer,context,"MECH_END mode=pose valid=%u missing=%u stored=%u",valid,missing,ok) && ok;
    return ok && missing==0;
}

bool mechanical_log_gait(const ActuatorDiagnostics *diagnostics, bool moving,
                         MechanicalLogWriter writer, void *context)
{
    if(!diagnostics || moving || !writer)return false;
    bool ok=emit(writer,context,"MECH_BEGIN mode=gait samples=%lu units=angle10,current_raw",
                 (unsigned long)diagnostics->total_samples);
    for(size_t i=0;i<ROBOT_JOINT_COUNT;i++) {
        const RobotJointConfig *c=&g_robot_joints[i];
        const ActuatorJointDiagnostics *j=&diagnostics->joints[i];
        if(j->sample_count==0) {
            ok=emit(writer,context,"MECH_UNSAMPLED id=%u",c->servo_id) && ok;
            continue;
        }
        ok=emit(writer,context,"MECH_TRACK id=%u n=%lu mean10=%ld peak=%u mv=%u lag=%u",
                c->servo_id,(unsigned long)j->sample_count,
                mean10(j->signed_error_sum_ticks,j->sample_count,c->direction),
                j->peak_position_error,j->minimum_voltage_mv,j->lag_samples) && ok;
        ok=emit(writer,context,"MECH_PHASE id=%u stance_n=%lu se10=%ld swing_n=%lu we10=%ld",
                c->servo_id,(unsigned long)j->stance_samples,
                mean10(j->stance_error_sum_ticks,j->stance_samples,c->direction),
                (unsigned long)j->swing_samples,
                mean10(j->swing_error_sum_ticks,j->swing_samples,c->direction)) && ok;
        const ActuatorTrackingSample *p=&j->peak_error_sample;
        ok=emit(writer,context,"MECH_PEAK id=%u cmd=%u pos=%u phase=%u stance=%u",
                c->servo_id,p->commanded_position,p->measured_position,p->gait_phase,p->stance) && ok;
        ok=emit(writer,context,"MECH_LAG id=%u cur=%d load=%d age=%u err=%d",
                c->servo_id,p->current,p->load,p->matched_target_age_frames,p->matched_target_error) && ok;
    }
    return emit(writer,context,"MECH_END mode=gait stored=%u",ok) && ok;
}

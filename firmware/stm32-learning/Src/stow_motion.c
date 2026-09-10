#include "robot.h"
#include "stow_control.h"
#include "sts3215.h"
#include "feetech_protocol.h"
#include <string.h>
#include <stdlib.h>

/* STS/SMS vendor tutorial: position mode 0, limits 9/11 both zero selects
 * multi-turn absolute positioning. Lock remains 1: do not persist settings.
 * Every setting is read back; unsupported firmware must fail before torque. */
static RobotResult bus_error(RobotController *r,unsigned id,ServoBusResult b) {
    r->last_failed_servo_id=id;r->last_bus_result=b;return ROBOT_BUS_ERROR;
}
static RobotResult snapshot(RobotController *r,Sts3215State s[12],float degrees[12],int32_t bias[12]) {
    for(unsigned i=0;i<12;i++) {
        ServoBusResult b=sts3215_read_state_raw(r->bus,g_robot_servo_ids[i],&s[i]);
        if(b!=SERVO_BUS_OK)return bus_error(r,g_robot_servo_ids[i],b);
        int32_t raw=stow_front(i)?stow_unwire(s[i].position):s[i].position;
        if(!stow_decode(i,raw,&degrees[i]))return ROBOT_POSITION_LIMIT;
        int32_t canonical;if(!stow_encode(i,degrees[i],&canonical))return ROBOT_POSITION_LIMIT;
        bias[i]=(int32_t)lroundf((raw-canonical)/4096.f)*4096;
        if(bias[i]<-4096 || bias[i]>4096 || bias[i]%4096 || (!stow_front(i) && bias[i]))return ROBOT_POSITION_LIMIT;
        if(s[i].hardware_error || s[i].temperature_c>=r->safety.limits.temperature_limit_c)return ROBOT_SAFETY_FAULT;
    }
    return ROBOT_OK;
}
RobotResult robot_stow_check(RobotController *r) {
    if(!r || !r->bus)return ROBOT_INVALID_ARGUMENT;
    for(unsigned i=1;i<=4;i+=3) {
        uint8_t id=g_robot_servo_ids[i],mode,resolution,bounds[4];
        ServoBusResult b=servo_bus_read(r->bus,id,33,&mode,1);
        if(b==SERVO_BUS_OK)b=servo_bus_read(r->bus,id,30,&resolution,1);
        if(b==SERVO_BUS_OK)b=servo_bus_read(r->bus,id,9,bounds,4);
        if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
        unsigned maximum=feetech_decode_u16(bounds+2);
        if(mode || resolution!=1 || bounds[0] || bounds[1] || (maximum && maximum!=4095)) {
            r->last_failed_servo_id=id;return ROBOT_CONFIG_ERROR;
        }
    }
    return ROBOT_OK;
}
static RobotResult limits(RobotController *r,bool extended) {
    for(unsigned i=1;i<=4;i+=3) {
        uint8_t id=g_robot_servo_ids[i],mode,resolution,old[4],check[4];
        ServoBusResult b=servo_bus_read(r->bus,id,33,&mode,1);
        if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
        b=servo_bus_read(r->bus,id,30,&resolution,1);
        if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
        if(mode!=0 || resolution!=1)return ROBOT_CONFIG_ERROR;
        b=servo_bus_read(r->bus,id,9,old,4);
        if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
        unsigned max=feetech_decode_u16(old+2);
        if(old[0] || old[1] || (max!=0 && max!=4095))return ROBOT_CONFIG_ERROR;
        const uint8_t desired[4]={0,0,extended?0:255,extended?0:15};
        if(!memcmp(old,desired,4))continue;
        b=sts3215_set_torque(r->bus,id,false);
        if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
        uint8_t lock=1;
        b=servo_bus_write(r->bus,id,55,&lock,1);
        if(b==SERVO_BUS_OK)b=servo_bus_write(r->bus,id,9,desired,4);
        if(b==SERVO_BUS_OK)b=servo_bus_read(r->bus,id,9,check,4);
        if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
        if(memcmp(check,desired,4))return ROBOT_CONFIG_ERROR;
    }
    return ROBOT_OK;
}
RobotResult robot_stow_check_modes(RobotController *r) {
    if(!r || r->stow_active || r->drive_active)return ROBOT_INVALID_ARGUMENT;
    for(unsigned i=0;i<12;i++) {
        uint8_t torque;
        ServoBusResult b=servo_bus_read(r->bus,g_robot_servo_ids[i],40,&torque,1);
        if(b!=SERVO_BUS_OK)return bus_error(r,g_robot_servo_ids[i],b);
        if(torque)return ROBOT_INVALID_ARGUMENT;
    }
    RobotResult result=robot_stow_check(r);
    if(result!=ROBOT_OK)return result;
    result=limits(r,true);
    RobotResult restored=limits(r,false);
    return result==ROBOT_OK?restored:result;
}
/* Bounded zero-displacement comparison. Only one front hip is energized;
 * all other axes must already be off. Always restore its original limits. */
RobotResult robot_stow_hold_check(RobotController *r,uint8_t id,bool extended,bool canonical) {
    if(!r || !r->bus || r->drive_active || (id!=2 && id!=5))return ROBOT_INVALID_ARGUMENT;
    r->stow_tracking_failure=false;
    for(unsigned i=0;i<12;i++) {
        uint8_t torque;ServoBusResult b=servo_bus_read(r->bus,g_robot_servo_ids[i],40,&torque,1);
        if(b!=SERVO_BUS_OK)return bus_error(r,g_robot_servo_ids[i],b);
        if(torque)return ROBOT_INVALID_ARGUMENT;
    }
    RobotResult result=robot_stow_check(r);if(result!=ROBOT_OK)return result;
    uint8_t old[4],check[4],lock=1;
    ServoBusResult b=servo_bus_read(r->bus,id,9,old,4);
    if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
    const uint8_t desired[4]={0,0,extended?0:255,extended?0:15};
    b=servo_bus_write(r->bus,id,55,&lock,1);
    if(b==SERVO_BUS_OK)b=servo_bus_write(r->bus,id,9,desired,4);
    if(b==SERVO_BUS_OK)b=servo_bus_read(r->bus,id,9,check,4);
    if(b!=SERVO_BUS_OK){result=bus_error(r,id,b);goto restore;}
    if(memcmp(check,desired,4)){result=ROBOT_CONFIG_ERROR;goto restore;}
    Sts3215State state;
    b=sts3215_read_state_raw(r->bus,id,&state);
    if(b!=SERVO_BUS_OK){result=bus_error(r,id,b);goto restore;}
    int32_t target=stow_unwire(state.position);
    if(target<64 || target>4031 || state.hardware_error || state.temperature_c>=r->safety.limits.temperature_limit_c) {
        result=ROBOT_POSITION_LIMIT;goto restore;
    }
    int32_t commanded=target;
    if(canonical) {
        float degrees;
        if(!extended || !stow_decode(id-1,target,&degrees) || !stow_encode(id-1,degrees,&commanded)) {
            result=ROBOT_POSITION_LIMIT;goto restore;
        }
    }
    r->stow_failure_target=target;r->stow_failure_actual=target;r->stow_failure_elapsed=0;
    uint8_t goal[7]={10,0,0,0,0,40,0};
    feetech_encode_u16(stow_wire(commanded),goal+1);
    b=servo_bus_write(r->bus,id,41,goal,7);
    if(b!=SERVO_BUS_OK){result=bus_error(r,id,b);goto restore;}
    r->motion_abort_requested=false;r->shared_idle=false;
    b=sts3215_set_torque(r->bus,id,true);
    if(b!=SERVO_BUS_OK){result=bus_error(r,id,b);goto restore;}
    for(unsigned elapsed=10;elapsed<=400;elapsed+=10) {
        HAL_Delay(10);
        if(r->motion_abort_requested){result=ROBOT_MOTION_ABORTED;goto restore;}
        b=sts3215_read_state_raw(r->bus,id,&state);
        if(b!=SERVO_BUS_OK){result=bus_error(r,id,b);goto restore;}
        int32_t actual=stow_unwire(state.position);
        if(abs(actual-target)>abs(r->stow_failure_actual-target)) {
            r->stow_failure_actual=actual;r->stow_failure_elapsed=elapsed;
        }
        if(abs(actual-target)>12 || state.hardware_error) {
            r->stow_tracking_failure=true;r->last_failed_servo_id=id;
            result=ROBOT_VERIFY_ERROR;goto restore;
        }
    }
restore:
    b=sts3215_set_torque(r->bus,id,false);
    if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
    b=servo_bus_write(r->bus,id,9,old,4);
    if(b==SERVO_BUS_OK)b=servo_bus_read(r->bus,id,9,check,4);
    if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
    if(memcmp(check,old,4))return ROBOT_CONFIG_ERROR;
    return result;
}
/* Resolve command turns by a bounded physical hold, not by assuming that
 * feedback contains the servo's internal turn count. No full sweep is allowed
 * until both hips have held their measured pose with the selected origin. */
static RobotResult reference_front(RobotController *r,unsigned i,int32_t *origin) {
    unsigned slot=i==1?0:1;uint8_t id=g_robot_servo_ids[i];
    int32_t candidates[4]={r->bus->front_origin_valid[slot]?r->bus->front_stow_origin[slot]:0,0,-4096,4096};
    for(unsigned c=0;c<4;c++) {
        bool duplicate=false;for(unsigned k=0;k<c;k++)if(candidates[k]==candidates[c])duplicate=true;
        if(duplicate)continue;
        ServoBusResult b=sts3215_set_torque(r->bus,id,false);
        if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
        Sts3215State state;float q;int32_t canonical;
        b=sts3215_read_state_raw(r->bus,id,&state);
        if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
        int32_t initial=stow_unwire(state.position);
        if(!stow_decode(i,initial,&q) || !stow_encode(i,q,&canonical))return ROBOT_POSITION_LIMIT;
        int32_t command=canonical+candidates[c];
        uint8_t goal[7]={10,0,0,0,0,40,0};
        feetech_encode_u16(stow_wire(command),goal+1);
        b=servo_bus_write(r->bus,id,41,goal,7);
        if(b==SERVO_BUS_OK)b=sts3215_set_torque(r->bus,id,true);
        if(b!=SERVO_BUS_OK){(void)sts3215_set_torque(r->bus,id,false);return bus_error(r,id,b);}
        bool held=true;
        for(unsigned elapsed=10;elapsed<=400;elapsed+=10) {
            HAL_Delay(10);
            if(r->motion_abort_requested){(void)sts3215_set_torque(r->bus,id,false);return ROBOT_MOTION_ABORTED;}
            b=sts3215_read_state_raw(r->bus,id,&state);
            if(b!=SERVO_BUS_OK){(void)sts3215_set_torque(r->bus,id,false);return bus_error(r,id,b);}
            int32_t raw=stow_unwire(state.position);
            if(state.hardware_error || state.temperature_c>=r->safety.limits.temperature_limit_c) {
                (void)sts3215_set_torque(r->bus,id,false);return ROBOT_SAFETY_FAULT;
            }
            if(abs(stow_feedback_error(raw,initial))>12) {held=false;break;}
        }
        if(held) {
            *origin=candidates[c];r->bus->front_stow_origin[slot]=*origin;
            r->bus->front_origin_valid[slot]=true;
            r->bus->front_position_bias[slot]=command-((command%4096+4096)%4096);
            return ROBOT_OK;
        }
        b=sts3215_set_torque(r->bus,id,false);
        if(b!=SERVO_BUS_OK)return bus_error(r,id,b);
        HAL_Delay(100);
    }
    r->last_failed_servo_id=id;return ROBOT_VERIFY_ERROR;
}
static RobotResult write_targets(RobotController *r,const int32_t pos[12]) {
    uint8_t items[12*7];
    for(unsigned i=0;i<12;i++) {
        if(pos[i]<-32767 || pos[i]>32767 || (!stow_front(i) && (pos[i]<0 || pos[i]>4095)))return ROBOT_POSITION_LIMIT;
        uint8_t *v=items+7*i;v[0]=30;
        feetech_encode_u16(stow_wire(pos[i]),v+1);
        feetech_encode_u16(0,v+3);feetech_encode_u16(800,v+5);
    }
    ServoBusResult b=servo_bus_sync_write(r->bus,41,7,g_robot_servo_ids,items,12);
    if(b==SERVO_BUS_OK)for(unsigned i=1;i<=4;i+=3) {
        r->bus->front_position_bias[i==1?0:1]=pos[i]-((pos[i]%4096+4096)%4096);
    }
    return b==SERVO_BUS_OK?ROBOT_OK:bus_error(r,254,b);
}
RobotResult robot_stow_probe(RobotController *r) {
    if(!r)return ROBOT_INVALID_ARGUMENT;
    if(r->stow_active)return ROBOT_OK;
    Sts3215State state[12];float q[12];int32_t bias[12];
    RobotResult result=snapshot(r,state,q,bias);
    if(result!=ROBOT_OK)return result;
    /* Rear hips + knees distinguish folded geometry from ordinary gait even
     * after both STM32 and servo turn counters have lost power. */
    float fraction=(40.f-q[7])/120.75f;
    bool folded=fraction>.03f && fraction<1.15f;
    for(unsigned i=0;i<12;i++) {
        float expected=stow_endpoint(i,false)+(stow_endpoint(i,true)-stow_endpoint(i,false))*fraction;
        if(fabsf(q[i]-expected)>18.f)folded=false;
    }
    /* A hand-folded or gravity-settled robot need not lie on the commanded
     * interpolation line. A front hip past -150deg must use the continuous
     * unfold path even when knees/rear hips settled independently. Snapshot
     * already validated every joint against the bounded Stow envelope. This
     * is a recovery classification, never an assertion of completed Stow. */
    if(q[1]<-150.f || q[4]<-150.f)folded=true;
    if(folded) {r->stow_active=true;r->stow_complete=false;r->shared_idle=false;}
    return ROBOT_OK;
}
RobotResult robot_stow_hold(RobotController *r) {
    Sts3215State state[12];float q[12];int32_t bias[12],target[12];
    RobotResult result=snapshot(r,state,q,bias);
    if(result!=ROBOT_OK) {(void)robot_relax(r);return result;}
    for(unsigned i=0;i<12;i++) {
        target[i]=state[i].position;
        if(stow_front(i)) {
            unsigned slot=i==1?0:1;
            if(!r->bus->front_origin_valid[slot]){(void)robot_relax(r);return ROBOT_POSITION_LIMIT;}
            if(!stow_encode(i,q[i],&target[i]))return ROBOT_POSITION_LIMIT;
            target[i]+=r->bus->front_stow_origin[slot];
        }
    }
    r->shared_idle=false;r->stow_complete=false;
    return write_targets(r,target); /* Do not re-enable torque after Relax. */
}
RobotResult robot_stow(RobotController *r,bool folded) {
    if(!r || r->drive_active)return ROBOT_INVALID_ARGUMENT;
    r->stow_tracking_failure=false;
    RobotResult result=robot_stow_check(r);
    if(result!=ROBOT_OK)return result;
    result=robot_stow_probe(r);
    if(result!=ROBOT_OK)return result;
    if(!r->stow_active) {
        result=robot_prepare_new_command(r);if(result!=ROBOT_OK)return result;
        /* Never begin a long sweep from Stand or an arbitrary pose. */
        result=robot_landing(r);if(result!=ROBOT_OK)return result;
    }
    r->stow_active=true;r->stow_complete=false;r->shared_idle=false;
    r->motion_abort_requested=false;
    result=limits(r,true);if(result!=ROBOT_OK)goto failed;
    Sts3215State state[12];float from[12],actual[12],degrees[12];int32_t bias[12],new_bias[12],pos[12];
    int32_t command_origin[2];
    result=reference_front(r,1,&command_origin[0]);if(result!=ROBOT_OK)goto failed;
    result=reference_front(r,4,&command_origin[1]);if(result!=ROBOT_OK)goto failed;
    result=snapshot(r,state,from,bias);if(result!=ROBOT_OK)goto failed;
    bias[1]=command_origin[0];bias[4]=command_origin[1];
    if(!stow_frame(from,folded,0,pos,degrees)){result=ROBOT_POSITION_LIMIT;goto failed;}
    for(unsigned i=0;i<12;i++)pos[i]+=bias[i];
    result=write_targets(r,pos);if(result!=ROBOT_OK)goto failed;
    for(unsigned i=0;i<12;i++) {
        if(r->motion_abort_requested){result=ROBOT_MOTION_ABORTED;goto failed;}
        ServoBusResult b=sts3215_set_torque(r->bus,g_robot_servo_ids[i],true);
        if(b!=SERVO_BUS_OK){result=bus_error(r,g_robot_servo_ids[i],b);goto failed;}
    }
    safety_clear(&r->safety);
    r->locomotion_fault=false;r->locomotion_fault_reason=ROBOT_OK;
    for(unsigned elapsed=20;elapsed<=STOW_DURATION_MS;elapsed+=20) {
        uint32_t start=HAL_GetTick();
        if(r->motion_abort_requested){result=ROBOT_MOTION_ABORTED;goto failed;}
        int16_t roll=0,pitch=0;
        bool valid=r->attitude_reader && r->attitude_reader(r->attitude_context,&roll,&pitch);
        if(!stow_attitude_ok(valid,roll,pitch)){result=valid?ROBOT_TILT_LIMIT:ROBOT_IMU_ERROR;goto failed;}
        if(!stow_frame(from,folded,elapsed,pos,degrees)){result=ROBOT_POSITION_LIMIT;goto failed;}
        for(unsigned i=0;i<12;i++)pos[i]+=bias[i];
        result=write_targets(r,pos);if(result!=ROBOT_OK)goto failed;
        /* One joint per frame; a developing stall is watched every frame. */
        unsigned i=(elapsed/20-1)%12;uint8_t watched=0;
        if(safety_watching(&r->safety,&watched))for(unsigned k=0;k<12;k++)if(g_robot_servo_ids[k]==watched)i=k;
        ServoBusResult b=sts3215_read_state_raw(r->bus,g_robot_servo_ids[i],&state[i]);
        if(b!=SERVO_BUS_OK){result=bus_error(r,g_robot_servo_ids[i],b);goto failed;}
        int32_t raw=stow_front(i)?stow_unwire(state[i].position):state[i].position;
        int32_t error=stow_front(i)?stow_feedback_error(raw,pos[i]):raw-pos[i];
        /* Compare physical position across modulo feedback boundaries only
         * after the command origin has passed the stationary hold above. */
        if(error>512 || error< -512){
            r->stow_tracking_failure=true;
            r->last_failed_servo_id=g_robot_servo_ids[i];
            r->stow_failure_elapsed=elapsed;
            r->stow_failure_target=pos[i];r->stow_failure_actual=raw;
            result=ROBOT_VERIFY_ERROR;goto failed;
        }
        SafetySample sample={g_robot_servo_ids[i],i/3,i%3+1,10000+pos[i],10000+pos[i]+error,
            state[i].load,state[i].current,state[i].temperature_c,state[i].hardware_error};
        if(safety_update(&r->safety,&sample,HAL_GetTick(),elapsed*1000/STOW_DURATION_MS)) {result=ROBOT_SAFETY_FAULT;goto failed;}
        uint32_t spent=HAL_GetTick()-start;if(spent<20)HAL_Delay(20-spent);
    }
    /* No final push/settling loop: release at the bounded pre-contact target. */
    if(folded) {
        result=snapshot(r,state,actual,new_bias);
        RobotResult released=robot_relax(r);
        if(result!=ROBOT_OK)return result;
        if(released!=ROBOT_OK)return released;
        for(unsigned i=0;i<12;i++)if(fabsf(actual[i]-stow_endpoint(i,true))>5.f)return ROBOT_VERIFY_ERROR;
        r->stow_complete=true;return ROBOT_OK;
    }
    result=snapshot(r,state,actual,new_bias);if(result!=ROBOT_OK)goto failed;
    for(unsigned i=0;i<12;i++)if(fabsf(actual[i]-stow_endpoint(i,false))>5.f){result=ROBOT_VERIFY_ERROR;goto failed;}
    /* Landing must remain supported. Releasing torque here let gravity move
     * the legs during mode restoration and falsely failed the final check.
     * Keep the temporary multi-turn setting and verified command origin;
     * ordinary motion APIs still enforce their software joint limits. */
    r->stow_active=false;r->stow_complete=false;
    return robot_hold(r);
failed:
    r->stow_complete=false;
    if(result==ROBOT_MOTION_ABORTED) (void)robot_stow_hold(r);
    else {(void)robot_relax(r);robot_latch_locomotion_fault(r,result);}
    return result;
}

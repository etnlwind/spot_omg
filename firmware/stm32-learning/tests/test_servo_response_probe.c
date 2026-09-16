#include "servo_response_probe.h"
#include "sts3215.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static RobotController r;
static ServoBus bus;
static uint32_t tick;
static uint8_t regs[12][8],original[12][8];
static uint16_t actual[12];
static unsigned writes,scenario,step_writes;
static int roll;
static unsigned temperature_reads,glitches;
enum { NORMAL, TORQUE_OFF, NOT_STAND, WRONG_PROFILE, BAD_BUS, TILT, ABORT,
       DRIFT, HOT, RESTORE_FAIL, FRONT_MULTITURN, MOVING_BASELINE, TEMP_GLITCH,
       TEMP_SINGLE_HOT, TEMP_SECOND_HOT, TEMP_READ_FAIL };
uint32_t HAL_GetTick(void){return tick;}
void HAL_Delay(uint32_t ms){tick+=ms;}
static bool imu(void *ctx,int16_t *a,int16_t *b){(void)ctx;*a=(int16_t)roll;*b=ROBOT_IMU_LEVEL_PITCH_TENTHS;return true;}
ServoBusResult servo_bus_read(ServoBus *b,uint8_t id,uint8_t addr,uint8_t *out,size_t n){
    (void)b;tick++;
    if(scenario==BAD_BUS && step_writes)return SERVO_BUS_TIMEOUT;
    if(addr==63 && n==1) {
        temperature_reads++;
        if(scenario==TEMP_READ_FAIL)return SERVO_BUS_TIMEOUT;
        *out=(scenario==HOT || scenario==TEMP_SINGLE_HOT)?75:35;
        return SERVO_BUS_OK;
    }
    assert(id>=1 && id<=12 && addr>=40 && addr+n<=48);
    memcpy(out,&regs[id-1][addr-40],n);
    if(scenario==WRONG_PROFILE && writes && id==12 && addr==41)out[0]=50;
    return SERVO_BUS_OK;
}
ServoBusResult servo_bus_write(ServoBus *b,uint8_t id,uint8_t addr,const uint8_t *p,size_t n){
    (void)b;tick++;
    /* Any accidental torque, other joint, or EEPROM write fails the test. */
    assert(id==12 && addr==41 && n==7);writes++;
    uint16_t old=(uint16_t)(regs[11][2]|(regs[11][3]<<8));
    uint16_t goal=(uint16_t)(p[1]|(p[2]<<8));
    if(goal!=old)step_writes++;
    if(scenario==RESTORE_FAIL && step_writes>=2 && writes>=4)return SERVO_BUS_TIMEOUT;
    memcpy(&regs[11][1],p,7);
    return SERVO_BUS_OK;
}
ServoBusResult sts3215_read_state(ServoBus *b,uint8_t id,Sts3215State *s){
    (void)b;tick++;
    if(scenario==BAD_BUS && step_writes)return SERVO_BUS_TIMEOUT;
    if(id==12) {
        uint16_t goal=(uint16_t)(regs[11][2]|(regs[11][3]<<8));
        if(actual[11]<goal)actual[11]++;else if(actual[11]>goal)actual[11]--;
        if(scenario==MOVING_BASELINE && writes)actual[11]=(uint16_t)(goal+(tick%2?4:0));
    }
    memset(s,0,sizeof(*s));s->position=actual[id-1];s->temperature_c=35;s->voltage_mv=12000;s->load=100;s->current=10;
    if(step_writes) {
        if(scenario==TILT)roll=ROBOT_IMU_LEVEL_ROLL_TENTHS+60;
        if(scenario==ABORT)r.motion_abort_requested=true;
        if(scenario==DRIFT && id==1)s->position+=25;
        if(scenario==HOT)s->temperature_c=75;
        if(id==12 && scenario>=TEMP_GLITCH && !glitches++){s->temperature_c=75;}
        if(id==12 && scenario==TEMP_SECOND_HOT)s->temperature_c=75;
    }
    return SERVO_BUS_OK;
}
static void setup(unsigned mode) {
    memset(&r,0,sizeof r);memset(regs,0,sizeof regs);memset(&bus,0,sizeof bus);
    scenario=mode;tick=writes=step_writes=temperature_reads=glitches=0;roll=ROBOT_IMU_LEVEL_ROLL_TENTHS;
    r.bus=&bus;r.attitude_reader=imu;r.shared_idle=true;r.locomotion_profile=22;safety_init(&r.safety,NULL);
    assert(robot_stand_targets(actual));
    for(unsigned i=0;i<12;i++) {
        regs[i][0]=1;regs[i][1]=(i%3==1)?254:50;
        regs[i][2]=(uint8_t)actual[i];regs[i][3]=(uint8_t)(actual[i]>>8);
        regs[i][6]=0x48;regs[i][7]=0x0d;
    }
    if(mode==TORQUE_OFF)regs[0][0]=0;
    if(mode==NOT_STAND)actual[2]+=100;
    if(mode==FRONT_MULTITURN){regs[1][3]+=16;regs[4][3]|=0x80;}
    memcpy(original,regs,sizeof regs);
}
static void untouched_others(void){assert(!memcmp(regs,original,11*8));for(unsigned j=0;j<12;j++)assert(regs[j][0]==original[j][0]);}
int main(void) {
    ServoResponseProbeReport d;
    for(unsigned acc=10;acc<=50;acc+=20) {
        setup(NORMAL);assert(robot_probe_rr_j3(&r,(uint8_t)acc,34,&d)==ROBOT_OK);
        assert(d.restored && d.stage==4 && !r.shared_idle && !memcmp(regs,original,sizeof regs));
        assert(r.joint_trace.command_count==3 && r.joint_trace.sample_count>200 && !r.joint_trace.full);
        assert(r.joint_trace.commands[1].target[11]==r.joint_trace.commands[0].target[11]+34);
        assert(r.joint_trace.commands[2].target[11]==r.joint_trace.commands[0].target[11]);
        for(unsigned i=0;i<r.joint_trace.sample_count;i++) {
            JointTraceSample *s=&r.joint_trace.samples[i];
            assert(s->end_ms>=s->begin_ms && s->begin_ms>=r.joint_trace.commands[s->command_index].end_ms);
        }
    }
    setup(FRONT_MULTITURN);assert(robot_probe_rr_j3(&r,10,23,&d)==ROBOT_OK);untouched_others();
    assert(r.joint_trace.commands[0].target[1]==(uint16_t)(original[1][2]|(original[1][3]<<8)));
    setup(TORQUE_OFF);assert(robot_probe_rr_j3(&r,10,23,&d)!=ROBOT_OK && writes==0);
    setup(NOT_STAND);assert(robot_probe_rr_j3(&r,10,23,&d)==ROBOT_STAND_REQUIRED && writes==0);
    setup(NORMAL);r.locomotion_fault=true;assert(robot_probe_rr_j3(&r,10,23,&d)==ROBOT_SAFETY_FAULT && writes==0);
    setup(NORMAL);assert(robot_probe_rr_j3(&r,50,35,&d)==ROBOT_INVALID_ARGUMENT && writes==0);
    setup(NORMAL);assert(robot_probe_rr_j3(&r,20,23,&d)==ROBOT_INVALID_ARGUMENT && writes==0);
    for(unsigned mode=WRONG_PROFILE;mode<=RESTORE_FAIL;mode++) {
        setup(mode);assert(robot_probe_rr_j3(&r,10,23,&d)!=ROBOT_OK);
        assert(!d.restored && !r.shared_idle);untouched_others();
        if(mode==BAD_BUS || mode==RESTORE_FAIL)assert(!d.hold_sent);
        else assert(d.hold_sent);
        if(mode==WRONG_PROFILE)assert(step_writes==0);
    }
    setup(MOVING_BASELINE);assert(robot_probe_rr_j3(&r,10,23,&d)==ROBOT_VERIFY_ERROR);assert(d.stage==1);untouched_others();
    setup(TEMP_GLITCH);assert(robot_probe_rr_j3(&r,10,23,&d)==ROBOT_OK);
    assert(d.temperature_suspects==1 && d.temperature_recovered==1 && temperature_reads==1 && d.restored);
    unsigned suspect_records=0;
    for(unsigned i=0;i<r.joint_trace.sample_count;i++)suspect_records+=r.joint_trace.samples[i].temperature>=70;
    assert(suspect_records==1);untouched_others();
    for(unsigned mode=TEMP_SINGLE_HOT;mode<=TEMP_READ_FAIL;mode++) {
        setup(mode);assert(robot_probe_rr_j3(&r,10,23,&d)!=ROBOT_OK);
        assert(d.temperature_suspects==1 && d.temperature_recovered==0 && !d.restored);untouched_others();
    }
    puts("RR J3 probe: bounded motion, raw-goal restore, sampling and fault exits passed");
}

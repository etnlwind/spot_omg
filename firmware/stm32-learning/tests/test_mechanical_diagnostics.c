#include "mechanical_diagnostics.h"
#include "sts3215.h"
#include "robot_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static char lines[80][96];
static size_t count,reads;
static int missing_id;
static bool fail_write;
static bool writer(void *context,const char *text) {
    (void)context;
    assert(strlen(text)<96 && count<80);
    strcpy(lines[count++],text);
    return !fail_write;
}
ServoBusResult sts3215_read_state(ServoBus *bus,uint8_t id,Sts3215State *state) {
    (void)bus;reads++;
    if(id==missing_id)return SERVO_BUS_TIMEOUT;
    const RobotJointConfig *c=&g_robot_joints[id-1];
    *state=(Sts3215State){.position=c->center+10*c->direction,
        .speed=-32768,.load=-1023,.voltage_mv=12500,.temperature_c=255,
        .hardware_error=255,.current=-32768};
    return SERVO_BUS_OK;
}
ServoBusResult servo_bus_read(ServoBus *bus,uint8_t id,uint8_t address,uint8_t *data,size_t size) {
    (void)bus;reads++;
    assert(address==40 && size==4);
    unsigned goal=g_robot_joints[id-1].center;
    data[0]=0;data[1]=0;data[2]=goal&255;data[3]=goal>>8;
    return SERVO_BUS_OK;
}
static void reset(void) {count=reads=0;missing_id=0;fail_write=false;}
int main(void) {
    ServoBus bus={0};
    reset();assert(!mechanical_log_capture(&bus,true,"jig_zero",writer,NULL));assert(!count && !reads);
    assert(!mechanical_log_capture(&bus,false,"bad label",writer,NULL));assert(!count && !reads);
    assert(!mechanical_log_capture(&bus,false,"12345678901234567",writer,NULL));
    assert(mechanical_log_capture(&bus,false,"jig_zero",writer,NULL));
    assert(count==50 && reads==24);
    for(unsigned i=0;i<12;i++) {
        assert(strstr(lines[1+i*4],"angle10=8"));
        assert(strstr(lines[4+i*4],"err10=-8"));
        assert(strstr(lines[4+i*4],"torque=0"));
    }
    reset();missing_id=5;
    assert(!mechanical_log_capture(&bus,false,"stand11",writer,NULL));
    assert(strstr(lines[count-1],"missing=1"));
    reset();fail_write=true;
    assert(!mechanical_log_capture(&bus,false,"stand11",writer,NULL));
    reset();ActuatorDiagnostics d;actuator_diagnostics_reset(&d);
    for(unsigned i=0;i<12;i++) {
        ActuatorTrackingSample s={.servo_id=i+1,.position_error=10*g_robot_joints[i].direction,
            .stance=true,.commanded_position=4095,.measured_position=0,.gait_phase=1000,
            .current=-32768,.load=-1023,.matched_target_age_frames=255,.matched_target_error=-32768};
        assert(actuator_diagnostics_update(&d,i,&s));
        s.position_error=-4*g_robot_joints[i].direction;s.stance=false;
        assert(actuator_diagnostics_update(&d,i,&s));
    }
    assert(!mechanical_log_gait(&d,true,writer,NULL));assert(count==0);
    assert(mechanical_log_gait(&d,false,writer,NULL));assert(count==50);
    for(unsigned i=0;i<12;i++) {
        assert(strstr(lines[1+i*4],"mean10=2"));
        assert(strstr(lines[2+i*4],"stance_n=1 se10=8 swing_n=1 we10=-3"));
    }
    reset();actuator_diagnostics_reset(&d);
    assert(mechanical_log_gait(&d,false,writer,NULL));assert(count==14);
    puts("mechanical diagnostics: idle guard, mirrored angles, phase stats, missing reads and storage failures passed");
}

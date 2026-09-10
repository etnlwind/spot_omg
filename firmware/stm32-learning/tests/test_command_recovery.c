#include "robot.h"
#include "sts3215.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static unsigned reads;
static int bad_id, hot_id, error_id, invalid_id;
ServoBusResult sts3215_read_state(ServoBus *b,uint8_t id,Sts3215State *s) {
    (void)b; reads++;
    if(id==bad_id)return SERVO_BUS_TIMEOUT;
    memset(s,0,sizeof(*s));
    s->position=id==invalid_id?65535:2048;
    s->temperature_c=id==hot_id?80:30;
    s->hardware_error=id==error_id?1:0;
    return SERVO_BUS_OK;
}
static void setup(RobotController *r, RobotResult reason) {
    memset(r,0,sizeof(*r)); safety_init(&r->safety,NULL);
    reads=0;bad_id=hot_id=error_id=invalid_id=0;
    r->drive_target_linear=600;r->drive_target_yaw=300;r->shared_idle=true;
    robot_latch_locomotion_fault(r,reason);
    assert(!r->shared_idle && !r->drive_target_linear && !r->drive_target_yaw && r->drive_stop_requested);
}
int main(void) {
    RobotController r;
    RobotResult reasons[]={ROBOT_TILT_LIMIT,ROBOT_IMU_ERROR,ROBOT_BUS_ERROR,ROBOT_MOTION_ABORTED,ROBOT_SAFETY_FAULT};
    for(unsigned i=0;i<sizeof(reasons)/sizeof(reasons[0]);i++) {
        setup(&r,reasons[i]);
        assert(robot_prepare_new_command(&r)==ROBOT_OK);
        assert(reads==12 && !r.locomotion_fault && !r.shared_idle);
        assert(r.drive_stop_requested && !r.drive_target_linear && !r.drive_target_yaw);
        /* Recovery cannot write motors: this test links only read_state. */
    }
    setup(&r,ROBOT_BUS_ERROR);bad_id=7;
    assert(robot_prepare_new_command(&r)==ROBOT_BUS_ERROR && r.locomotion_fault);
    assert(reads==7 && r.last_failed_servo_id==7);
    bad_id=0;reads=0;assert(robot_prepare_new_command(&r)==ROBOT_OK && reads==12);
    setup(&r,ROBOT_TILT_LIMIT);hot_id=3;
    assert(robot_prepare_new_command(&r)==ROBOT_SAFETY_FAULT && r.locomotion_fault);
    setup(&r,ROBOT_TILT_LIMIT);error_id=8;
    assert(robot_prepare_new_command(&r)==ROBOT_SAFETY_FAULT && r.locomotion_fault);
    setup(&r,ROBOT_TILT_LIMIT);invalid_id=12;
    assert(robot_prepare_new_command(&r)==ROBOT_SAFETY_FAULT && r.locomotion_fault);
    setup(&r,ROBOT_SAFETY_FAULT);r.safety.fault=SAFETY_FAULT_OVERHEAT;hot_id=3;
    assert(robot_prepare_new_command(&r)==ROBOT_SAFETY_FAULT && safety_is_faulted(&r.safety));
    hot_id=0;assert(robot_prepare_new_command(&r)==ROBOT_OK && !safety_is_faulted(&r.safety));
    setup(&r,ROBOT_BUS_ERROR);r.drive_active=true;
    assert(robot_prepare_new_command(&r)==ROBOT_INVALID_ARGUMENT && reads==0 && r.locomotion_fault);
    puts("fresh command recovery passed");return 0;
}

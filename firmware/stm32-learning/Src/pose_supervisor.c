#include "robot.h"
#include "sts3215.h"
#include "pose_control.h"
#include <string.h>

/* Provisional conservative envelopes, not learned load/contact classifiers.
 * Never infer contact or impact from Euler angles alone. No torque-off here. */
#define POSE_LEAD_TICKS 40U
#define POSE_PROBE_LEAD_TICKS 120U
#define POSE_PROBE_WAIT_MS 300U
#define POSE_ARRIVAL_TICKS 24U
#define POSE_PROGRESS_TICKS 3U
#define POSE_NO_PROGRESS_MS 1800U
#define POSE_FRESH_MS 500U
#define POSE_READ_ATTEMPTS 3U
#define POSE_VOLTAGE_MIN_MV 10500U
static unsigned gap(unsigned a,unsigned b){return a>b?a-b:b-a;}
static unsigned mag(int v){return v<0?(unsigned)-v:(unsigned)v;}
const char *pose_decision_name(PoseDecision r) {
 static const char *names[]={"idle","moving","waiting","complete","no-progress",
 "obstruction-suspected","feedback-lost","write-failed","encoder-invalid",
 "low-voltage","servo-fault","imu-unavailable","unstable","cancelled","hold-failed","complete-residual","load-supported"};
 return (unsigned)r<sizeof(names)/sizeof(names[0])?names[r]:"unknown";
}
static RobotResult read_all(RobotController *r,Sts3215State state[12],uint32_t stamp[12]) {
 for(unsigned i=0;i<12;i++) {
  ServoBusResult b=SERVO_BUS_TIMEOUT;
  for(unsigned k=0;k<POSE_READ_ATTEMPTS;k++) {
   b=sts3215_read_state(r->bus,g_robot_servo_ids[i],&state[i]);
   if(b==SERVO_BUS_OK)break;
   r->pose_diagnostics.retries++;
   if(r->motion_abort_requested)break;
   HAL_Delay(10);
  }
  if(b!=SERVO_BUS_OK){r->last_failed_servo_id=g_robot_servo_ids[i];r->last_bus_result=b;
   r->pose_diagnostics.reason=POSE_FEEDBACK_LOST;return ROBOT_BUS_ERROR;}
  /* Pause target progression while confirming an abnormal health sample.
   * A valid packet alone does not establish persistent hardware failure. */
  if(state[i].hardware_error || state[i].temperature_c>=70 || state[i].voltage_mv<10500) {
   r->pose_diagnostics.suspect_id=g_robot_servo_ids[i];
   r->pose_diagnostics.suspect_temperature=state[i].temperature_c;
   r->pose_diagnostics.suspect_hardware=state[i].hardware_error;
   Sts3215State repeated;HAL_Delay(15);
   b=sts3215_read_state(r->bus,g_robot_servo_ids[i],&repeated);
   if(b!=SERVO_BUS_OK){r->pose_diagnostics.reason=POSE_FEEDBACK_LOST;
    r->last_failed_servo_id=g_robot_servo_ids[i];r->last_bus_result=b;return ROBOT_BUS_ERROR;}
   if(!repeated.hardware_error && repeated.temperature_c<70 && repeated.voltage_mv>=10500)r->pose_diagnostics.health_recovered++;
   state[i]=repeated;
  }
  stamp[i]=HAL_GetTick();
  if(state[i].position<g_robot_joints[i].minimum || state[i].position>g_robot_joints[i].maximum) {
   r->last_failed_servo_id=g_robot_servo_ids[i];r->pose_diagnostics.reason=POSE_ENCODER_INVALID;return ROBOT_POSITION_LIMIT;
  }
 }
 for(unsigned i=0;i<12;i++)if(HAL_GetTick()-stamp[i]>POSE_FRESH_MS) {
  r->pose_diagnostics.reason=POSE_FEEDBACK_LOST;return ROBOT_BUS_ERROR;
 }
 return ROBOT_OK;
}
static RobotResult imu(RobotController *r,int16_t *roll,int16_t *pitch) {
 if(!r->attitude_reader || !r->attitude_reader(r->attitude_context,roll,pitch)) {
  r->pose_diagnostics.reason=POSE_ATTITUDE_UNAVAILABLE;return ROBOT_IMU_ERROR;
 }
 if(mag((int)*roll-ROBOT_IMU_LEVEL_ROLL_TENTHS)>150 ||
    mag((int)*pitch-ROBOT_IMU_LEVEL_PITCH_TENTHS)>150) {
  r->pose_diagnostics.reason=POSE_UNSTABLE;return ROBOT_TILT_LIMIT;
 }
 return ROBOT_OK;
}
static void record(RobotController *r,const Sts3215State s[12],const uint16_t cmd[12],
                   unsigned i,uint32_t begin,int16_t roll,int16_t pitch) {
 PoseDiagnostics *d=&r->pose_diagnostics;
 d->elapsed_ms=HAL_GetTick()-begin;
 d->trace[d->next]=(PoseObservation){d->elapsed_ms,s[i].position,cmd[i],gap(s[i].position,cmd[i]),
  s[i].voltage_mv,s[i].load,s[i].current,roll,pitch,g_robot_servo_ids[i],(uint8_t)d->reason};
 d->next=(d->next+1)%POSE_TRACE_COUNT;if(d->count<POSE_TRACE_COUNT)d->count++;
}
/* On a diagnosed stop, use fresh positions to remove the remaining position
 * error, preserving torque state. If feedback cannot be obtained, send NO
 * guessed target. Existing bounded targets remain in the servo; this cannot
 * guarantee physical rest when the bus is lost. Never claim a verified hold. */
static void arrest(RobotController *r) {
 PoseDecision cause=r->pose_diagnostics.reason;
 uint8_t failed=r->last_failed_servo_id;ServoBusResult bus=r->last_bus_result;
 uint8_t sid=r->pose_diagnostics.suspect_id,st=r->pose_diagnostics.suspect_temperature,sh=r->pose_diagnostics.suspect_hardware;
 Sts3215State s[12];uint32_t stamps[12];uint16_t target[12];
 if(read_all(r,s,stamps)!=ROBOT_OK){r->pose_diagnostics.hold_verified=3;goto done;}
 for(unsigned i=0;i<12;i++)target[i]=s[i].position;
 if(sts3215_sync_positions(r->bus,g_robot_servo_ids,target,12)!=SERVO_BUS_OK){
  r->pose_diagnostics.hold_verified=3;goto done;}
 r->pose_diagnostics.hold_verified=1;HAL_Delay(80);
 if(read_all(r,s,stamps)!=ROBOT_OK){r->pose_diagnostics.hold_verified=3;goto done;}
 for(unsigned i=0;i<12;i++)if(gap(s[i].position,target[i])>POSE_ARRIVAL_TICKS)goto done;
 r->pose_diagnostics.hold_verified=2;
done:
 r->pose_diagnostics.suspect_id=sid;r->pose_diagnostics.suspect_temperature=st;r->pose_diagnostics.suspect_hardware=sh;
 r->pose_diagnostics.reason=cause;r->last_failed_servo_id=failed;r->last_bus_result=bus;
}
RobotResult robot_supervised_pose(RobotController *r,const uint16_t to[12],bool stand_requested) {
 if(!r || !r->bus || !to || r->stow_active || (r->drive_active && !r->drive_pose_entry))return ROBOT_INVALID_ARGUMENT;
 memset(&r->pose_diagnostics,0,sizeof(r->pose_diagnostics));
 r->shared_idle=false;r->motion_abort_requested=false;robot_support_reset(r);
 Sts3215State s[12];uint32_t stamp[12],progress_at[12];
 SupportWindow loaded[12]={0};uint8_t unsettled[12]={0};
 uint16_t from[12],cmd[12],anchor[12],previous[12],lead[12];int16_t roll=0,pitch=0;
 RobotResult result=robot_reference_pose(r);if(result!=ROBOT_OK)return result;
 result=read_all(r,s,stamp);if(result!=ROBOT_OK)return result;
 result=imu(r,&roll,&pitch);if(result!=ROBOT_OK)return result;
 for(unsigned i=0;i<12;i++) {
  if(to[i]<g_robot_joints[i].minimum || to[i]>g_robot_joints[i].maximum)return ROBOT_POSITION_LIMIT;
  if(s[i].hardware_error || s[i].temperature_c>=70) {
   r->pose_diagnostics.reason=POSE_SERVO_FAULT;return ROBOT_SAFETY_FAULT;
  }
  if(s[i].voltage_mv<POSE_VOLTAGE_MIN_MV) {
   r->pose_diagnostics.reason=POSE_LOW_VOLTAGE;return ROBOT_SAFETY_FAULT;
  }
  lead[i]=POSE_LEAD_TICKS;
  from[i]=cmd[i]=anchor[i]=previous[i]=s[i].position;progress_at[i]=HAL_GetTick();
 }
 uint32_t duration=pose_transition_duration(from,to,stand_requested);
 /* Restore the original single profiled goal for Landing -> Stand only.
  * Other poses retain measured-progress interpolation. */
 const bool fast=pose_fast_stand(from,stand_requested); if(!duration)duration=300;
 r->pose_diagnostics.nominal_ms=fast?0:duration;
 uint32_t begin=HAL_GetTick(),phase=0,last=begin,settled_since=0;
 /* Seed positions before enabling any axis. Failed enable never rolls other
  * supporting axes back to torque-off. */
 if(sts3215_sync_move(r->bus,g_robot_servo_ids,from,12,fast?r->profile_speed:300,fast?r->profile_acceleration:30)!=SERVO_BUS_OK){
  r->pose_diagnostics.reason=POSE_WRITE_FAILED;return ROBOT_BUS_ERROR;
 }
 for(unsigned i=0;i<12;i++)if(sts3215_set_torque(r->bus,g_robot_servo_ids[i],true)!=SERVO_BUS_OK) {
  r->last_failed_servo_id=g_robot_servo_ids[i];r->pose_diagnostics.reason=POSE_WRITE_FAILED;
  arrest(r);return ROBOT_BUS_ERROR;
 }
 if(fast) {
  if(r->motion_abort_requested || (r->drive_pose_entry && r->drive_stop_requested)) {
   r->pose_diagnostics.reason=POSE_CANCELLED;arrest(r);return ROBOT_MOTION_ABORTED;
  }
  if(sts3215_sync_move(r->bus,g_robot_servo_ids,to,12,r->profile_speed,r->profile_acceleration)!=SERVO_BUS_OK) {
   r->pose_diagnostics.reason=POSE_WRITE_FAILED;arrest(r);return ROBOT_BUS_ERROR;
  }
  memcpy(cmd,to,sizeof cmd);phase=duration;
 }
 for(;;) {
  if(r->motion_abort_requested || (r->drive_pose_entry && r->drive_stop_requested)){r->pose_diagnostics.reason=POSE_CANCELLED;result=ROBOT_MOTION_ABORTED;break;}
  result=read_all(r,s,stamp);if(result!=ROBOT_OK)break;
  result=imu(r,&roll,&pitch);if(result!=ROBOT_OK)break;
  uint32_t now=HAL_GetTick();unsigned worst=0;bool caught=true,arrived=true,residual_stable=true,load_supported=true;
  r->pose_diagnostics.reason=POSE_MOVING;
  for(unsigned i=0;i<12;i++) {
   unsigned error=gap(cmd[i],s[i].position);
   if(error>gap(cmd[worst],s[worst].position))worst=i;
   if(error>POSE_ARRIVAL_TICKS)caught=false;
   if(gap(to[i],s[i].position)>POSE_ARRIVAL_TICKS)arrived=false;
   /* Field observation: a healthy low-effort joint can retain 26 ticks of
    * static error. Do not confuse that with a blocked large excursion. A
    * residual completion must be inside the SAME 40-tick command envelope,
    * stationary for 500ms, low effort, valid voltage/IMU, and explicitly logged. */
   if(gap(to[i],s[i].position)>POSE_LEAD_TICKS ||
      gap(previous[i],s[i].position)>2 ||
      mag(s[i].load)>=r->safety.limits.load_magnitude ||
      mag(s[i].current)>=r->safety.limits.current_magnitude)residual_stable=false;
   previous[i]=s[i].position;
   /* Reject unhealthy hardware before treating any movement as progress. */
   if(s[i].hardware_error || s[i].temperature_c>=70) {
    r->pose_diagnostics.reason=POSE_SERVO_FAULT;result=ROBOT_SAFETY_FAULT;worst=i;break;
   }
   if(s[i].voltage_mv<POSE_VOLTAGE_MIN_MV){r->pose_diagnostics.reason=POSE_LOW_VOLTAGE;result=ROBOT_SAFETY_FAULT;worst=i;break;}
   if(phase>=duration && gap(to[i],s[i].position)<=SUPPORT_MAX_ERROR) {
    SupportDecision support=support_update(&loaded[i],now,to[i],s[i].position,roll,pitch,
        s[i].load,s[i].current,s[i].voltage_mv,s[i].temperature_c,s[i].hardware_error);
    if(support!=SUPPORT_STABLE)load_supported=false;
    if(support==SUPPORT_STABLE)unsettled[i]=0;
    if(support==SUPPORT_ADJUSTING && ++unsettled[i]>=3) {
     r->pose_diagnostics.reason=POSE_UNSTABLE;result=ROBOT_VERIFY_ERROR;worst=i;break;
    }
    if(support>=SUPPORT_DRIFT){r->pose_diagnostics.reason=POSE_NO_PROGRESS;result=ROBOT_VERIFY_ERROR;worst=i;break;}
   } else load_supported=false;
   bool towards=gap(anchor[i],to[i])>=gap(s[i].position,to[i])+POSE_PROGRESS_TICKS;
   if(towards || error<=POSE_ARRIVAL_TICKS){anchor[i]=s[i].position;progress_at[i]=now;}
   /* At the 40-tick envelope a loaded position servo may need more error
    * to generate starting effort. One bounded probe per axis per command:
    * same trajectory/direction, no arrival relaxation, no timeout reset.
    * Low measured effort is NOT proof that the path is unobstructed. */
   if(lead[i]==POSE_LEAD_TICKS && error>=POSE_LEAD_TICKS-4 &&
      now-progress_at[i]>=POSE_PROBE_WAIT_MS && phase<duration &&
      mag(s[i].load)<r->safety.limits.load_magnitude &&
      mag(s[i].current)<r->safety.limits.current_magnitude) {
    lead[i]=POSE_PROBE_LEAD_TICKS;
    r->pose_diagnostics.effort_probes++;
    r->pose_diagnostics.probe_mask|=(uint16_t)(1U<<i);
   }
   if(error>POSE_ARRIVAL_TICKS &&
      ((phase<duration && gap(cmd[i],to[i])>POSE_ARRIVAL_TICKS) ||
       gap(to[i],s[i].position)>SUPPORT_MAX_ERROR ||
       mag(s[i].current)>=r->safety.limits.current_magnitude) &&
      now-progress_at[i]>=POSE_NO_PROGRESS_MS) {
    bool effort=mag(s[i].load)>=r->safety.limits.load_magnitude || mag(s[i].current)>=r->safety.limits.current_magnitude;
    r->pose_diagnostics.reason=effort?POSE_OBSTRUCTION_SUSPECTED:POSE_NO_PROGRESS;
    result=ROBOT_VERIFY_ERROR;worst=i;break;
   }
  }
  if(result!=ROBOT_OK){r->last_failed_servo_id=g_robot_servo_ids[worst];record(r,s,cmd,worst,begin,roll,pitch);break;}
  if(phase>=duration && load_supported) {
   r->pose_diagnostics.reason=POSE_LOAD_SUPPORTED;record(r,s,cmd,worst,begin,roll,pitch);return ROBOT_OK;
  }
  if(phase>=duration && arrived){r->pose_diagnostics.reason=POSE_COMPLETE;record(r,s,cmd,worst,begin,roll,pitch);return ROBOT_OK;}
  if(phase>=duration && residual_stable) {
   if(!settled_since)settled_since=now;
   if(now-settled_since>=500) {
    r->pose_diagnostics.reason=POSE_COMPLETE_RESIDUAL;record(r,s,cmd,worst,begin,roll,pitch);return ROBOT_OK;
   }
  } else settled_since=0;
  if(!caught){r->pose_diagnostics.reason=POSE_WAITING;r->pose_diagnostics.waits++;}
  record(r,s,cmd,worst,begin,roll,pitch);
  uint32_t step=now-last;last=now;if(step>100)step=100;if(step<20)step=20;
  if(phase<duration) {
   uint32_t next=phase+step;if(next>duration)next=duration;
   uint16_t candidate[12];pose_frame(from,to,duration,next,candidate);
   /* Advance all joints on ONE common phase. Halve phase advancement until
    * every axis has a bounded remaining excursion, never clip per-axis paths. */
   bool fits=false;
   while(next>phase) {
    fits=true;pose_frame(from,to,duration,next,candidate);
    for(unsigned i=0;i<12;i++)if(gap(candidate[i],s[i].position)>lead[i])fits=false;
    if(fits)break;
    next=phase+(next-phase)/2;
   }
   if(fits) {
    if(sts3215_sync_positions(r->bus,g_robot_servo_ids,candidate,12)!=SERVO_BUS_OK){
     r->pose_diagnostics.reason=POSE_WRITE_FAILED;result=ROBOT_BUS_ERROR;break;
    }
    memcpy(cmd,candidate,sizeof(cmd));phase=next;
   }
  }
  HAL_Delay(20);
 }
 r->pose_diagnostics.elapsed_ms=HAL_GetTick()-begin;
 arrest(r);return result;
}

void robot_support_reset(RobotController *r) {
 memset(r->support_windows,0,sizeof(r->support_windows));r->support_targets_valid=false;
 r->support_index=0;r->support_stable_mask=0;r->support_decision=SUPPORT_OBSERVING;r->support_id=0;
}
bool robot_support_observe(RobotController *r,const uint16_t targets[12]) {
 unsigned i=r->support_index;r->support_index=(i+1)%12;
 Sts3215State state;ServoBusResult bus=sts3215_read_state(r->bus,g_robot_servo_ids[i],&state);
 SupportDecision d=SUPPORT_OBSERVING;
 if(bus!=SERVO_BUS_OK)d=SUPPORT_FEEDBACK;
 else {
  if(state.hardware_error || state.temperature_c>=70 || state.voltage_mv<10500 || mag(state.current)>=700) {
   /* Confirm once with no new target issued; do not hide persistent faults. */
   bus=sts3215_read_state(r->bus,g_robot_servo_ids[i],&state);
  }
  if(bus!=SERVO_BUS_OK)d=SUPPORT_FEEDBACK;
  else if(state.position>4095)d=SUPPORT_FEEDBACK;
  else if(r->support_targets_valid) d=support_update(&r->support_windows[i],HAL_GetTick(),
    r->support_targets[i],state.position,r->balance_last_roll_error_tenths,
    r->balance_last_pitch_error_tenths,state.load,state.current,state.voltage_mv,
    state.temperature_c,state.hardware_error);
 }
 if(d>=SUPPORT_DRIFT) {
  r->support_decision=d;r->support_id=g_robot_servo_ids[i];r->support_event++;
  r->shared_idle=false; /* Freeze correction targets, preserve torque and last supporting goal. */
  return false;
 }
 if(d==SUPPORT_STABLE)r->support_stable_mask|=(uint16_t)(1U<<i);
 else if(d==SUPPORT_ADJUSTING)r->support_stable_mask&=(uint16_t)~(1U<<i);
 r->support_decision=r->support_stable_mask==0xfff?SUPPORT_STABLE:SUPPORT_ADJUSTING;
 memcpy(r->support_targets,targets,sizeof(r->support_targets));r->support_targets_valid=true;
 return true;
}

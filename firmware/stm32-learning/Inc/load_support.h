#ifndef LOAD_SUPPORT_H
#define LOAD_SUPPORT_H
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
/* Measured complete robot mass is 2754g. Mass is context, not an exemption:
 * no force estimate is possible from mass alone without lever arms/contact. */
#define ROBOT_MEASURED_MASS_G 2754U
#define SUPPORT_MAX_ERROR 120U
#define SUPPORT_WINDOW_MS 1200U
typedef enum { SUPPORT_OBSERVING, SUPPORT_STABLE, SUPPORT_ADJUSTING,
 SUPPORT_DRIFT=100, SUPPORT_EFFORT, SUPPORT_VOLTAGE, SUPPORT_HEALTH,
 SUPPORT_FEEDBACK, SUPPORT_ATTITUDE } SupportDecision;
static inline const char *support_name(SupportDecision d) {
 switch(d) {
 case SUPPORT_OBSERVING:return "observing";case SUPPORT_STABLE:return "load-supported";
 case SUPPORT_ADJUSTING:return "adjusting";case SUPPORT_DRIFT:return "position-drifting";
 case SUPPORT_EFFORT:return "excess-current";case SUPPORT_VOLTAGE:return "low-voltage";
 case SUPPORT_HEALTH:return "servo-health";case SUPPORT_FEEDBACK:return "feedback-lost";
 case SUPPORT_ATTITUDE:return "attitude-unavailable";default:return "unknown";
 }
}
typedef struct {
 uint32_t start,last;
 int target,actual,error,roll,pitch,min_actual,max_actual,min_roll,max_roll,min_pitch,max_pitch;
 bool initialized;
} SupportWindow;
static inline SupportDecision support_update(SupportWindow *w,uint32_t now,
 int target,int actual,int roll,int pitch,int load,int current,
 unsigned voltage,unsigned temperature,unsigned hardware) {
 int error=abs(target-actual);
 if(hardware || temperature>=70)return SUPPORT_HEALTH;
 if(voltage<10500)return SUPPORT_VOLTAGE;
 if(abs(current)>=700)return SUPPORT_EFFORT;
 if((unsigned)error>SUPPORT_MAX_ERROR)return SUPPORT_DRIFT;
 if(!w->initialized || now-w->last>500) {
  *w=(SupportWindow){now,now,target,actual,error,roll,pitch,actual,actual,roll,roll,pitch,pitch,true};return SUPPORT_OBSERVING;
 }
 w->last=now;
 if(roll<w->min_roll)w->min_roll=roll;
 if(roll>w->max_roll)w->max_roll=roll;
 if(pitch<w->min_pitch)w->min_pitch=pitch;
 if(pitch>w->max_pitch)w->max_pitch=pitch;
 if(actual<w->min_actual)w->min_actual=actual;
 if(actual>w->max_actual)w->max_actual=actual;
 if(now-w->start<SUPPORT_WINDOW_MS)return SUPPORT_OBSERVING;
 bool fixed=abs(target-w->target)<=4;
 bool stable=w->max_actual-w->min_actual<=8 && w->max_roll-w->min_roll<=10 && w->max_pitch-w->min_pitch<=10;
 bool growing=fixed && error-w->error>=12 && abs(actual-w->actual)>=12;
 /* High load with stable position is consistent with support, not proof of contact.
  * A fixed target with increasing error is deterioration, even at low load. */
 SupportDecision decision=growing?SUPPORT_DRIFT:fixed&&stable?SUPPORT_STABLE:SUPPORT_ADJUSTING;
 (void)load;
 *w=(SupportWindow){now,now,target,actual,error,roll,pitch,actual,actual,roll,roll,pitch,pitch,true};return decision;
}
#endif

#ifndef TILT_PAUSE_H
#define TILT_PAUSE_H
#include <stdbool.h>
/* Explicit new command only. Never rebase on a heartbeat or an idle tick.
 * Preserve real attitude for stabilization; only the stop envelope changes.
 * A retry permits the existing tilt plus the normal per-axis tilt limit in
 * that direction. Opposite tilt retains the normal limit. Level restores it. */
typedef struct { int roll, pitch; bool active; } TiltPause;
static inline bool tilt_pause_exceeded(TiltPause *s,int roll,int pitch,int limit) {
    if(s->active && roll>=-120 && roll<=120 && pitch>=-120 && pitch<=120)s->active=false;
    int r=s->active?s->roll:0,p=s->active?s->pitch:0;
    return roll>(r>0?r:0)+limit || roll<(r<0?r:0)-limit ||
           pitch>(p>0?p:0)+limit || pitch<(p<0?p:0)-limit;
}
#endif

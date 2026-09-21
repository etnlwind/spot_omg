#ifndef IMU_AUTO_RECOVERY_H
#define IMU_AUTO_RECOVERY_H
#include <stdbool.h>
#include <stdint.h>
/* One attempt per failed episode; repeated errors do not postpone its deadline. */
typedef struct { uint32_t failed_at; bool pending, attempted; } ImuAutoRecovery;
static inline void imu_auto_failed(ImuAutoRecovery *s,uint32_t now) {
    if(!s->pending && !s->attempted){s->failed_at=now;s->pending=true;}
}
static inline void imu_auto_healthy(ImuAutoRecovery *s) {
    s->pending=false;s->attempted=false;
}
static inline bool imu_auto_take(ImuAutoRecovery *s,uint32_t now,bool idle) {
    if(!idle || !s->pending || (uint32_t)(now-s->failed_at)<3000U)return false;
    s->pending=false;s->attempted=true;
    return true;
}
#endif

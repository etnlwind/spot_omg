#ifndef IMU_TRACE_H
#define IMU_TRACE_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define IMU_TRACE_CAPACITY 512U
/* First ~10 seconds at 50Hz. MCU observation time, not sensor sample time.
 * RAM only: no extra sensor reads, UART writes, allocation or motion. */
typedef struct {
    uint32_t time_ms;
    int16_t roll10,pitch10;
    uint16_t phase1000,rate1000;
    uint8_t valid,fault;
} ImuTraceSample;
typedef struct {
    ImuTraceSample samples[IMU_TRACE_CAPACITY];
    uint16_t count;
    bool armed,full;
} ImuTrace;
static inline void imu_trace_arm(ImuTrace *t) {
    t->count=0;t->armed=true;t->full=false;
}
static inline void imu_trace_record(ImuTrace *t,ImuTraceSample sample) {
    if(!t->armed)return;
    if(t->count>=IMU_TRACE_CAPACITY){t->armed=false;t->full=true;return;}
    t->samples[t->count++]=sample;
    if(t->count==IMU_TRACE_CAPACITY){t->armed=false;t->full=true;}
}
#endif

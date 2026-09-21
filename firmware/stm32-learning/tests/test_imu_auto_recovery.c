#undef NDEBUG
#include <assert.h>
#include <stdint.h>
#include "imu_auto_recovery.h"
int main(void) {
 ImuAutoRecovery s={0};
 assert(!imu_auto_take(&s,9000,true));
 imu_auto_failed(&s,100);
 imu_auto_failed(&s,2000); /* repeated failures do not postpone the timer */
 assert(!imu_auto_take(&s,3099,true));
 assert(!imu_auto_take(&s,3100,false)); /* never reset during motion */
 assert(imu_auto_take(&s,3101,true));
 imu_auto_failed(&s,3200);
 assert(!imu_auto_take(&s,10000,true)); /* failed attempt is not a retry loop */
 imu_auto_healthy(&s); /* verified manual/automatic recovery rearms a new episode */
 imu_auto_failed(&s,UINT32_MAX-999U);
 assert(!imu_auto_take(&s,1999,true));
 assert(imu_auto_take(&s,2000,true)); /* tick wrap */
 imu_auto_healthy(&s);imu_auto_failed(&s,10);imu_auto_healthy(&s);
 assert(!imu_auto_take(&s,4000,true)); /* successful manual recovery cancels timer */
 return 0;
}

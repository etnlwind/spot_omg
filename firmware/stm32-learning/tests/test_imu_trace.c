#include "imu_trace.h"
#include "attitude_control.h"
#include <assert.h>
int main(void){
 static ImuTrace t;AttitudeControl attitude={0};
 ImuTraceSample sample={.time_ms=0xfffffff0U,.roll10=130,.valid=1};
 imu_trace_record(&t,sample);assert(t.count==0);
 imu_trace_arm(&t);
 sample.fault=attitude_update(&attitude,true,130,0);imu_trace_record(&t,sample);
 sample.time_ms=4;sample.fault=attitude_update(&attitude,true,130,0);imu_trace_record(&t,sample);
 assert(t.count==2 && t.samples[0].time_ms==0xfffffff0U && t.samples[1].fault==2);
 t.armed=false;imu_trace_record(&t,sample);assert(t.count==2);
 imu_trace_arm(&t);assert(t.count==0 && !t.full && t.armed);
 sample.valid=0;sample.fault=1;
 for(unsigned i=0;i<IMU_TRACE_CAPACITY;i++){sample.time_ms=i*20;imu_trace_record(&t,sample);}
 assert(t.full && !t.armed && t.count==512 && !t.samples[0].valid);
 sample.time_ms=99999;imu_trace_record(&t,sample);
 assert(t.samples[0].time_ms==0 && t.samples[511].time_ms==10220);
 return 0;
}

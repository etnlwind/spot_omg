#include <assert.h>
#include "battery_telemetry.h"
int main(void) {
 BatteryTelemetry b={0};
 battery_telemetry_sample(&b,10900);battery_telemetry_sample(&b,10300);battery_telemetry_sample(&b,10800);
 assert(!battery_telemetry_take(&b,999));assert(battery_telemetry_take(&b,1000)==10300);
 assert(!battery_telemetry_take(&b,2000));
 battery_telemetry_sample(&b,0);assert(!battery_telemetry_take(&b,2000));
 battery_telemetry_sample(&b,12000);assert(battery_telemetry_take(&b,2000)==12000);
 battery_telemetry_sample(&b,65535);assert(!battery_telemetry_take(&b,3000));
 b.sent_ms=UINT32_MAX-500;battery_telemetry_sample(&b,11000);
 assert(!battery_telemetry_take(&b,498));assert(battery_telemetry_take(&b,499)==11000);
 BatteryTelemetry shared={0};uint32_t usb=0,ble=0;
 battery_telemetry_sample(&shared,10300);
 assert(battery_telemetry_poll(&shared,1000,&usb)==10300);
 assert(battery_telemetry_poll(&shared,1020,&ble)==10300);
 assert(!battery_telemetry_poll(&shared,1040,&usb));
 battery_telemetry_sample(&shared,12000);
 assert(battery_telemetry_poll(&shared,2000,&ble)==12000);
 assert(battery_telemetry_poll(&shared,2020,&usb)==12000);
 return 0;
}

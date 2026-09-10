#include <assert.h>
#include "drive_watchdog.h"
int main(void) {
    assert(!drive_watchdog_due(800,0,800,false));
    assert(drive_watchdog_due(801,0,800,false));
    assert(!drive_watchdog_due(5000,0,800,true));
    assert(!drive_watchdog_due(100,UINT32_MAX-100,800,false));
    assert(drive_watchdog_due(1000,UINT32_MAX-100,800,false));
    return 0;
}

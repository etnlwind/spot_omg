from pathlib import Path
import subprocess
from servo.host_build import build_executable

def test_delayed_one_shot_recovery_scheduler(tmp_path):
    root=Path(__file__).resolve().parents[1]
    exe=build_executable([root/'tests/test_imu_auto_recovery.c'],root/'Inc',tmp_path/'auto-recovery',extra=['-Wall','-Wextra','-Werror'])
    subprocess.run([str(exe)],check=True)

def test_auto_recovery_runs_only_in_foreground_idle(tmp_path):
    root=Path(__file__).resolve().parents[1]
    source=(root/'Src/app_console.c').read_text()
    fn=source[source.index('void app_console_auto_imu_recovery('):source.index('static void command_i2cscan(')]
    unit=tmp_path/'auto.c'
    unit.write_text('''
#undef NDEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "imu_auto_recovery.h"
static ImuAutoRecovery imu_auto;
typedef struct {bool drive_active,walk_probe_active,stow_active,gait_diagnostics_active;} RobotController;
typedef struct {bool present;} Bno055;
typedef struct {RobotController *robot;Bno055 *imu055;bool control_active;} AppConsole;
static uint32_t now;
static unsigned recoveries;
static uint32_t HAL_GetTick(void){return now;}
static void write_text(AppConsole*c,const char*t){(void)c;(void)t;}
static bool command_imurecover(AppConsole*c){(void)c;recoveries++;return false;}
static void flight_log_appendf(const char*f,...){(void)f;}
'''+fn+'''
int main(void){
 RobotController r={0};Bno055 imu={true};AppConsole c={&r,&imu,false};
 imu_auto_failed(&imu_auto,0);now=2999;app_console_auto_imu_recovery(&c);assert(!recoveries);
 now=3000;r.drive_active=true;app_console_auto_imu_recovery(&c);assert(!recoveries);
 r.drive_active=false;r.stow_active=true;app_console_auto_imu_recovery(&c);assert(!recoveries);
 r.stow_active=false;r.gait_diagnostics_active=true;app_console_auto_imu_recovery(&c);assert(!recoveries);
 r.gait_diagnostics_active=false;r.walk_probe_active=true;app_console_auto_imu_recovery(&c);assert(!recoveries);
 r.walk_probe_active=false;c.control_active=true;app_console_auto_imu_recovery(&c);assert(!recoveries);
 c.control_active=false;app_console_auto_imu_recovery(&c);assert(recoveries==1);
 now=6000;app_console_auto_imu_recovery(&c);assert(recoveries==1);
 return 0;
}
''')
    exe=build_executable([unit],root/'Inc',tmp_path/'auto',extra=['-Wall','-Wextra','-Werror'])
    subprocess.run([str(exe)],check=True)

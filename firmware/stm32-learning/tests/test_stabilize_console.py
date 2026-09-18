"""Execute the real @B parser/RX ISR/service bodies with a UART HAL mock.

This extracts functions from app_console.c at test time; it does not copy the
parser implementation into a mock. Unrelated console commands, the physical
UART interrupt scheduler, and the blocking robot drive are outside this host
test. line_ready=True models the foreground being owned by robot_drive().
"""
from pathlib import Path
import re
import shutil
import subprocess
import sys

import pytest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT.parents[1]/'tools/servo_tool'))
from servo.host_build import build_executable

PREFIX = r'''
#undef NDEBUG
#define HAL_MAX_DELAY 0xffffffffu
#define HAL_OK 0
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "body_stabilizer.h"
#include "battery_telemetry.h"
/* ATTITUDE_PD_TYPES */
typedef struct { volatile uint32_t DR; } UART_Registers;
typedef struct { UART_Registers *Instance; } UART_HandleTypeDef;
typedef int Bno055;
typedef int Bno086;
typedef struct {
    volatile bool stabilization_enabled;
    bool motion_abort_requested, drive_stop_requested;
    int locomotion_profile;
    uint32_t drive_sequence;
    int16_t linear,yaw;
    AttitudePd attitude_pd;
    BatteryTelemetry battery_telemetry;
    void (*realtime_service)(void);
} RobotController;
#define RESET 0
#define UART_FLAG_TXE 1
#define ROBOT_DRIVE_INPUT_LIMIT 1000
static bool inside_isr;
static unsigned transmissions,rearms,clear_errors,drive_updates,drive_stops;
static uint32_t last_timeout;
static UART_HandleTypeDef *last_tx_uart;
static char last_message[256];
static void (*tx_injection)(void);
#define __HAL_UART_GET_FLAG(uart,flag) ((void)(uart),(void)(flag),1)
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart),++clear_errors)
static uint32_t tick=700;
static uint32_t HAL_GetTick(void) { return tick; }
static int HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *b,uint16_t n) {
    assert(u && b && n==1);++rearms;return 0;
}
static int HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *b,uint16_t n,uint32_t timeout) {
    assert(!inside_isr);assert(u && b && n<sizeof(last_message));
    ++transmissions;last_timeout=timeout;last_tx_uart=u;
    memcpy(last_message,b,n);last_message[n]='\0';
    if (tx_injection) { void (*callback)(void)=tx_injection;tx_injection=NULL;callback(); }
    return 0;
}
static int locomotion_profile_id(const char *name) { return !strcmp(name,"attitudepd")?42:43; }
/* LOCOMOTION_PD_HELPER */
static void robot_request_motion_abort(RobotController *r) { if(r)r->motion_abort_requested=true; }
static bool robot_drive_update_realtime(RobotController *r,uint32_t sequence,int16_t x,int16_t y,uint32_t at) {
    assert(r && at==700);++drive_updates;r->drive_sequence=sequence;r->linear=x;r->yaw=y;return true;
}
static bool robot_drive_stop_realtime(RobotController *r,uint32_t sequence,uint32_t at) {
    assert(r && at==700);++drive_stops;r->drive_sequence=sequence;r->drive_stop_requested=true;return true;
}
'''

HARNESS = r'''
static AppConsole console,wifi_console;
#define other wifi_console
static RobotController robot;
static UART_Registers registers[2];
static UART_HandleTypeDef uart[2];
/* ROUND_ROBIN_SERVICE */
/* BUDGETED_SERVICE */
static void feed(AppConsole *c,const char *text,size_t length) {
    for(size_t i=0;i<length;++i) {
        c->rx_byte=(uint8_t)text[i];inside_isr=true;
        app_console_on_rx_complete(c,c->uart);inside_isr=false;
    }
}
static void send(AppConsole *c,const char *text) { feed(c,text,strlen(text)); }
static void setup(void) {
    memset(&robot,0,sizeof(robot));memset(&console,0,sizeof(console));memset(&other,0,sizeof(other));
    uart[0].Instance=&registers[0];uart[1].Instance=&registers[1];
    robot.locomotion_profile=42;
    body_stabilizer_reset(&robot.attitude_pd.body);
    transmissions=rearms=clear_errors=drive_updates=drive_stops=0;
    tx_injection=NULL;inside_isr=false;tick=700;
    app_console_init(&console,&uart[0],&robot,NULL,NULL,NULL);
    app_console_init(&other,&uart[1],&robot,NULL,NULL,NULL);
}
static void busy_foreground(AppConsole *c) {
    strcpy(c->line,"drive");c->line_length=5;c->line_ready=true;
}
static void foreground_ack(AppConsole *c,bool enabled) {
    const unsigned before=transmissions;
    app_console_service_realtime(c);
    assert(transmissions==before+1 && last_timeout==10);
    assert(last_tx_uart==c->uart);
    assert(strstr(last_message,enabled ? "enabled=1" : "enabled=0"));
    assert(strstr(last_message,"rate_hz=50"));
}
static void basic(void) {
    busy_foreground(&console);console.echo_enabled=true;
    send(&console,"status\n");assert(!strcmp(console.line,"drive"));
    send(&console,"@B1");assert(!robot.stabilization_enabled && !console.stabilize_reply_pending);
    send(&console,"\r\n");assert(robot.stabilization_enabled && console.stabilize_reply_pending);
    assert(console.line_ready && console.line_length==5 && !strcmp(console.line,"drive"));
    assert(transmissions==0 && drive_updates==0 && drive_stops==0);
    foreground_ack(&console,true);assert(strstr(last_message,"status=no-sample"));
    app_console_service_realtime(&console);assert(transmissions==1);
    send(&console,"@B0\n");assert(!robot.stabilization_enabled);foreground_ack(&console,false);
    send(&console,"@B2\n");assert(!robot.stabilization_enabled);foreground_ack(&console,false);
    send(&console,"@B 1   \n");assert(robot.stabilization_enabled);foreground_ack(&console,true);
    send(&console,"@B2\n");assert(robot.stabilization_enabled);foreground_ack(&console,true);
}
static void invalid_text(void) {
    const char *bad[]={"@B-1\n","@B3\n","@B2147483647\n","@B2147483648\n",
        "@B-2147483648\n","@B999999999999999999999999\n","@B1 0\n","@B1junk\n",
        "@B1.0\n","@B+1\n","@B  \n","@B\n","@b1\n","@B1\t\n",
        "@B0x1\n","@B1@B0\n","@B-\n","@B 1 2\r\n"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(*bad);++i) for(unsigned state=0;state<2;++state) {
        setup();busy_foreground(&console);robot.stabilization_enabled=state;
        send(&console,bad[i]);assert(robot.stabilization_enabled==(bool)state);
        assert(!console.stabilize_reply_pending && transmissions==0);
    }
}
static void binary_trailing(void) {
    /* RX must not let an embedded C-string terminator hide trailing bytes. */
    const char malformed[]={'@','B','1','\0','j','u','n','k','\n'};
    busy_foreground(&console);feed(&console,malformed,sizeof(malformed));
    assert(!robot.stabilization_enabled && !console.stabilize_reply_pending);
}
static void overflow_and_recovery(void) {
    busy_foreground(&console);
    send(&console,"@B1");
    for(unsigned i=0;i<2*APP_CONSOLE_REALTIME_CAPACITY;++i)send(&console," ");
    assert(console.realtime_overflow);
    send(&console,"\n");assert(!robot.stabilization_enabled && !console.stabilize_reply_pending);
    assert(console.realtime_length==0 && !console.realtime_overflow);
    send(&console,"@B1\n");assert(robot.stabilization_enabled);foreground_ack(&console,true);
}
static void stop_independent(void) {
    busy_foreground(&console);
    send(&console,"@D 77 500 -200\n");assert(drive_updates==1);
    send(&console,"@B1\n");assert(robot.drive_sequence==77 && robot.linear==500 && robot.yaw==-200);
    send(&console,"@S 78\n");assert(drive_stops==1 && robot.drive_stop_requested);
    send(&console,"@B0\n@B1\n");assert(robot.drive_stop_requested && robot.drive_sequence==78);
    send(&console,"\003");assert(robot.motion_abort_requested);
    send(&console,"@B0\n@B2\n");assert(robot.motion_abort_requested && robot.drive_stop_requested);
    assert(transmissions==0);foreground_ack(&console,false);
}
static void reset_recovery(void) {
    busy_foreground(&console);send(&console,"@B");
    const unsigned before=rearms;
    app_console_on_uart_error(&console,&uart[0]);
    assert(rearms==before+1 && clear_errors==1 && console.realtime_length==0 && !console.line_ready);
    send(&console,"1\n");assert(!robot.stabilization_enabled && !console.stabilize_reply_pending);
    send(&console,"@B1\n");assert(robot.stabilization_enabled && console.stabilize_reply_pending);
    app_console_init(&console,&uart[0],&robot,NULL,NULL,NULL);
    assert(!console.stabilize_reply_pending && !console.realtime_length && !console.line_ready);
    assert(robot.stabilization_enabled); /* console re-init must not reset robot's chosen setting */
    send(&console,"@B0\n");foreground_ack(&console,false);
}
static void dual_console(void) {
    busy_foreground(&console);busy_foreground(&other);
    send(&other,"@B1\n");assert(other.stabilize_reply_pending && !console.stabilize_reply_pending);
    app_console_service_realtime(&console);assert(transmissions==0);
    send(&console,"@B0\n");assert(!robot.stabilization_enabled);
    foreground_ack(&other,false);foreground_ack(&console,false); /* latest shared state wins */
    unsigned before=rearms;console.rx_byte='@';
    app_console_on_rx_complete(&console,&uart[1]);assert(rearms==before && !console.realtime_length);
    app_console_on_uart_error(&console,&uart[1]);assert(clear_errors==0);
    send(&other,"@B2\n");foreground_ack(&other,false);
}
static void inject_off(void) { send(&console,"@B0\n"); }
static void service_interleave(void) {
    busy_foreground(&console);send(&console,"@B1\n");
    tx_injection=inject_off;foreground_ack(&console,true);
    assert(!robot.stabilization_enabled && console.stabilize_reply_pending);
    foreground_ack(&console,false);assert(!console.stabilize_reply_pending);
    app_console_service_realtime(&console);assert(transmissions==2);
}
static void status_format(void) {
    robot.attitude_pd.body.initialized=true;
    robot.attitude_pd.body.diagnostics.status=BODY_STABILIZER_AXES_UNVERIFIED;
    send(&console,"@B2\n");foreground_ack(&console,false);
    assert(strstr(last_message,"status=axes-unverified"));
    robot.attitude_pd.ik_failed=true;
    send(&console,"@B2\n");foreground_ack(&console,false);
    assert(strstr(last_message,"status=ik-infeasible"));
    robot.locomotion_profile=7;send(&console,"@B2\n");foreground_ack(&console,false);
    assert(strstr(last_message,"status=other-policy"));
    app_console_service_realtime(NULL);
}
static void round_robin(void) {
    UART_HandleTypeDef *previous=NULL;
    for(unsigned i=0;i<8;++i) {
        console.stabilize_reply_pending=true;other.stabilize_reply_pending=true;
        service_realtime_console();
        assert(transmissions==i+1 && last_timeout==10);
        assert(console.stabilize_reply_pending!=other.stabilize_reply_pending);
        assert(last_tx_uart!=previous);previous=last_tx_uart;
    }
    console.stabilize_reply_pending=false;other.stabilize_reply_pending=true;
    service_realtime_console();assert(transmissions==9 && last_tx_uart==other.uart);
    service_realtime_console();assert(transmissions==9);
}
static void frame_budget(void) {
    robot.realtime_service=service_realtime_console;
    console.stabilize_reply_pending=true;other.stabilize_reply_pending=true;
    maybe_service(&robot,689U); /* next frame at709, 9ms remain: defer */
    assert(transmissions==0 && console.stabilize_reply_pending && other.stabilize_reply_pending);
    maybe_service(&robot,690U); /* 10ms remain: at most one ACK */
    assert(transmissions==1 && console.stabilize_reply_pending!=other.stabilize_reply_pending);
    maybe_service(&robot,680U);assert(transmissions==1); /* deadline reached */
    maybe_service(&robot,670U);assert(transmissions==1); /* already late */
    robot.realtime_service=NULL;maybe_service(&robot,700U);assert(transmissions==1);
    robot.realtime_service=service_realtime_console;tick=UINT32_MAX-3U;
    maybe_service(&robot,UINT32_MAX-3U);assert(transmissions==2); /* wrap,20ms remain */
}
int main(int argc,char **argv) {
    assert(argc==2);setup();
    if(!strcmp(argv[1],"basic"))basic();
    else if(!strcmp(argv[1],"invalid"))invalid_text();
    else if(!strcmp(argv[1],"binary"))binary_trailing();
    else if(!strcmp(argv[1],"overflow"))overflow_and_recovery();
    else if(!strcmp(argv[1],"stop"))stop_independent();
    else if(!strcmp(argv[1],"reset"))reset_recovery();
    else if(!strcmp(argv[1],"dual"))dual_console();
    else if(!strcmp(argv[1],"interleave"))service_interleave();
    else if(!strcmp(argv[1],"status"))status_format();
    else if(!strcmp(argv[1],"roundrobin"))round_robin();
    else if(!strcmp(argv[1],"budget"))frame_budget();
    else assert(0);
    puts("stabilize console contract passed");return 0;
}
'''


@pytest.fixture(scope="module")
def console_executable(tmp_path_factory):
    source = (PROJECT / "Src/app_console.c").read_text()
    header = (PROJECT / "Inc/app_console.h").read_text()
    adapter = (PROJECT / "Inc/attitude_pd.h").read_text()
    main = (PROJECT / "Src/main.c").read_text()
    robot = (PROJECT / "Src/robot.c").read_text()
    # Whole live production function ranges, not reimplemented parser logic.
    macros = "\n".join(re.findall(r"^#define APP_CONSOLE_.*$", header, re.M))
    structure = header[header.index("typedef struct"):header.index("} AppConsole;") + len("} AppConsole;")]
    echo = source[source.index("static void write_wire("):source.index("static bool parse_u32(")]
    service_start = source.index("void app_console_service_realtime(")
    service = source[service_start:source.index("static void command_stabilize(", service_start)]
    realtime = source[source.index("static bool realtime_next_i32("):source.index("void app_console_poll(")]
    adapter_types = adapter[adapter.index("typedef struct"):adapter.index("/* Refine")]
    round_start = main.index("static void service_realtime_console(void)")
    round_end = main.index("\n}", round_start) + len("\n}")
    round_service = main[round_start:round_end]
    budget_start = robot.index("if(robot->realtime_service &&")
    budget_end = robot.index("robot->realtime_service();", budget_start) + len("robot->realtime_service();")
    budget_service = "static void maybe_service(RobotController *robot,uint32_t deadline) {\n" + robot[budget_start:budget_end] + "\n}"
    frame_macro = re.search(r"^#define ROBOT_TROT_FRAME_MS\s+.*$", robot, re.M).group(0)
    directory = tmp_path_factory.mktemp("stabilize-console")
    unit = directory / "console.c"
    prefix = PREFIX.replace("/* ATTITUDE_PD_TYPES */", adapter_types)
    locomotion = (PROJECT / "Inc/locomotion.h").read_text()
    helper = locomotion[locomotion.index("static inline bool locomotion_is_attitude_pd"):locomotion.index("static inline bool locomotion_has_gait_stand")]
    prefix = prefix.replace("/* LOCOMOTION_PD_HELPER */", helper)
    harness = HARNESS.replace("/* ROUND_ROBIN_SERVICE */", round_service).replace("/* BUDGETED_SERVICE */", budget_service)
    unit.write_text("\n".join([prefix, macros, frame_macro, structure, echo, service, realtime, harness]))
    output = build_executable([unit], PROJECT/'Inc', directory/'console',
                              extra=['-Wall','-Wextra','-Werror'])
    return output


@pytest.mark.parametrize("case", [
    "basic", "invalid", "binary", "overflow", "stop", "reset", "dual", "interleave", "status", "roundrobin", "budget",
])
def test_realtime_stabilize_contract(console_executable, case):
    subprocess.run([str(console_executable), case], check=True, capture_output=True, text=True)


def test_service_is_registered_in_drive_foreground_only():
    main = (PROJECT / "Src/main.c").read_text()
    robot = (PROJECT / "Src/robot.c").read_text()
    console = (PROJECT / "Src/app_console.c").read_text()
    assert "robot.realtime_service = service_realtime_console;" in main
    # Runtime tests above execute the live deadline guard and round-robin body.
    # Keep registration/order checks here: ACK service must precede the deadline
    # increment and must never migrate into the UART receive interrupt.
    service_start = robot.index("if(robot->realtime_service &&")
    service_end = robot.index("robot->realtime_service();", service_start)
    guard = robot[service_start:service_end]
    assert "deadline+ROBOT_TROT_FRAME_MS-HAL_GetTick()" in guard
    assert ">=10" in guard
    assert service_end < robot.index("deadline+=ROBOT_TROT_FRAME_MS;", service_start)
    rx = console[console.index("void app_console_on_rx_complete("):console.index("void app_console_on_uart_error(")]
    assert "app_console_service_realtime(" not in rx
    assert "HAL_UART_Transmit(" not in rx

"""Execute production transport/RX/poll functions with fake UART and commands.

Motion physics and servo control are outside this test. Control request
mailboxes, output queues, frame ordering and interrupt parsing are real C.
"""
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/servo_tool'))
from servo.host_build import build_executable


def test_control_completion_and_next_command_bypass_console_backlog(tmp_path):
    source=(ROOT/'firmware/stm32-learning/Src/app_console.c').read_text()
    header=(ROOT/'firmware/stm32-learning/Inc/app_console.h').read_text()
    macros='\n'.join(re.findall(r'^#define APP_CONSOLE_.*$',header,re.M))
    structure=header[header.index('typedef struct'):header.index('} AppConsole;')+len('} AppConsole;')]
    wire=source[source.index('static void write_wire('):source.index('static bool parse_u32(')]
    rx=source[source.index('static bool realtime_next_i32('):source.index('void app_console_print_help(')]
    prefix=r'''
#undef NDEBUG
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
typedef struct {uint32_t DR;} Registers;
typedef struct {Registers* Instance;} UART_HandleTypeDef;
typedef int Bno055;
typedef int Bno086;
typedef struct {bool stabilization_enabled; unsigned support_event, support_decision, support_id;} RobotController;
#define HAL_MAX_DELAY 0xffffffffu
#define HAL_OK 0
#define RESET 0
#define UART_FLAG_TXE 1
#define ROBOT_DRIVE_INPUT_LIMIT 1000
#define __HAL_UART_GET_FLAG(u,f) 1
#define __HAL_UART_CLEAR_OREFLAG(u) ((void)(u))
static char output[65536];static size_t written;static unsigned executed,updates,stops,aborts;
static uint32_t HAL_GetTick(void){return 100;}
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __enable_irq(void){}
static int HAL_UART_Transmit(UART_HandleTypeDef*u,uint8_t*b,uint16_t n,uint32_t t){
 (void)u;(void)t;assert(written+n<sizeof(output));memcpy(output+written,b,n);written+=n;output[written]=0;return 0;}
static int HAL_UART_Receive_IT(UART_HandleTypeDef*u,uint8_t*b,uint16_t n){(void)u;(void)b;(void)n;return 0;}
static void robot_request_motion_abort(RobotController*r){(void)r;++aborts;}
static int robot_drive_update_realtime(RobotController*r,uint32_t s,int16_t l,int16_t y,uint32_t t){(void)r;(void)s;(void)l;(void)y;(void)t;++updates;return 1;}
static int robot_drive_stop_realtime(RobotController*r,uint32_t s,uint32_t t){(void)r;(void)s;(void)t;++stops;return 1;}
static const char* support_name(unsigned x){(void)x;return "ok";}
'''
    stubs=r'''
static void command_sync_state(AppConsole*c){write_text(c,"$SPOTSTATE pose=stand torque=on safety=ok\r\n");}
static void execute_line(AppConsole*c){
 ++executed;
 if(!strncmp(c->line,"drive ",6)){
   write_text(c,"$SPOTDRIVE started seq=1\r\n");
   write_text(c,"$SPOTDRIVE stopped reason=ok\r\nOK\r\n");
   for(unsigned i=0;i<150;++i)write_text(c,"Gait diagnostics still being printed\r\n");
 }else command_sync_state(c);
}
void app_console_service_realtime(AppConsole*c){flush_console_log(c);}
void app_console_print_prompt(AppConsole*c);
'''
    main=r'''
static void receive(AppConsole*c,const char*s){while(*s){c->rx_byte=(uint8_t)*s++;app_console_on_rx_complete(c,c->uart);}}
int main(void){
 Registers reg={0};UART_HandleTypeDef uart={&reg};RobotController robot={0};AppConsole c={0};bool flag=false;
 app_console_init(&c,&uart,&robot,NULL,NULL,&flag);
 receive(&c,"@C 7 drive 1000 0 1\n");assert(c.control_pending && !written && !executed);
 app_console_poll(&c);assert(executed==1 && !c.control_pending && !c.control_active);
 assert(strstr(output,"\0367 DATA $SPOTDRIVE stopped") && strstr(output,"\0367 DONE \037"));
 assert(!strstr(output,"Gait diagnostics"));assert(c.log_head!=c.log_tail && !c.log_dropped);
 size_t first=written;
 receive(&c,"@C 8 syncstate\n");app_console_poll(&c);
 assert(executed==2 && strstr(output+first,"\0368 DONE \037"));
 assert(!strstr(output,"Gait diagnostics")); /* next command beats 5KB of logs */
 receive(&c,"@C 8 syncstate\n");assert(!c.control_pending); /* duplicate */
 receive(&c,"@C 9 syncstate\n");receive(&c,"\003");assert(!c.control_pending && aborts==1);
 receive(&c,"@C 10 syncstate\n");app_console_on_uart_error(&c,&uart);assert(!c.control_pending);
 c.line_ready=true;receive(&c,"@D 2 1000 0\n@S 3\n");assert(updates==1 && stops==1);c.line_ready=false;
 while(c.log_head!=c.log_tail)flush_console_log(&c);
 assert(strstr(output,"Gait diagnostics still being printed"));
 for(unsigned i=0;i<400;++i)write_text(&c,"012345678901234567890123456789\r\n");
 assert(c.log_dropped>0);
 receive(&c,"@C 11 syncstate\n");app_console_poll(&c);assert(strstr(output,"\03611 DONE \037"));
 while(c.log_head!=c.log_tail)flush_console_log(&c);flush_console_log(&c);
 assert(strstr(output,"[LOG overflow:"));
 puts("control completion independent of console backlog: passed");return 0;
}
'''
    unit=tmp_path/'transport.c'
    unit.write_text('\n'.join([prefix,macros,structure,wire,stubs,rx,main]))
    exe=build_executable([unit],ROOT/'firmware/stm32-learning/Inc',tmp_path/'transport',extra=['-Wall','-Wextra','-Werror'])
    subprocess.run([str(exe)],check=True,capture_output=True,text=True)


def test_bridge_routes_control_frames_while_console_overflows(tmp_path):
    from servo.host_build import _compiler_command
    source=(ROOT/'firmware/esp32-c3-mini/esp32_c3_stm32_uart_bridge/esp32_c3_stm32_uart_bridge.ino').read_text()
    routing=source[source.index('static void routeUartByte('):source.index('void setup()')]
    globals_=source[source.index('static BLECharacteristic* controlTx'):source.index('static constexpr uint32_t STM32_BAUD')]
    prefix=r'''
#undef NDEBUG
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
namespace std {template<typename T>T min(T a,T b){return a<b?a:b;}}
static constexpr size_t BLE_NOTIFY_CHUNK=180;
static bool clientConnected=true;
static char received[4096];static size_t receivedCount,packets;
class BLECharacteristic {public:
 void setValue(uint8_t*b,size_t n){assert(n<=180 && receivedCount+n<sizeof(received));memcpy(received+receivedCount,b,n);receivedCount+=n;}
 void indicate(){++packets;}
};
'''
    main=r'''
int main(){
 (void)lastLogNotify;
 blePayload=180;
 BLECharacteristic channel;controlTx=&channel;
 const char* text="ID11 before";while(*text)routeUartByte(*text++);
 char frame[704];frame[0]=0x1e;memset(frame+1,'x',702);frame[703]=0x1f;
 for(unsigned i=0;i<sizeof(frame);++i)routeUartByte(frame[i]);
 text=" after\r\n# ";while(*text)routeUartByte(*text++);
 assert(receivedCount==sizeof(frame) && !memcmp(received,frame,sizeof(frame)) && packets==4);
 assert(logHead==21 && !memcmp(logQueue,"ID11 before after\r\n# ",21));
 for(unsigned i=0;i<20000;++i)routeUartByte('L');
 assert(logLost>0);receivedCount=0;
 for(unsigned i=0;i<sizeof(frame);++i)routeUartByte(frame[i]);
 assert(receivedCount==sizeof(frame) && !memcmp(received,frame,sizeof(frame)));
 receivedCount=0;packets=0;blePayload=20;
 for(unsigned i=0;i<sizeof(frame);++i)routeUartByte(frame[i]);
 assert(receivedCount==sizeof(frame) && packets==36 && !memcmp(received,frame,sizeof(frame)));
 return 0;
}
'''
    unit=tmp_path/'bridge.cpp';unit.write_text(prefix+globals_+routing+main)
    output=tmp_path/('bridge.exe' if sys.platform=='win32' else 'bridge')
    subprocess.run(_compiler_command()+['-std=c++17','-Wall','-Wextra','-Werror',str(unit),'-o',str(output)],check=True,capture_output=True,text=True)
    subprocess.run([str(output)],check=True,capture_output=True,text=True)

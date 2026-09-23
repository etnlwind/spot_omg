/* Production transport + production gait retry helpers; peripheral/time model only.
 * This is NOT a measurement of ISR WCET, electrical behavior or robot physics. */
#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#undef assert
#define assert(x) do {if(!(x)){fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);exit(1);}} while(0)
#include "robot.h"
#include "sts3215.h"
#include "feetech_protocol.h"

uint32_t SystemCoreClock=84000000;
static USART_TypeDef regs;
static UART_HandleTypeDef uart={&regs};
static ServoBus bus;
static uint64_t us;
static uint32_t clock_base,masked;
static unsigned requests,tx_bytes,legacy_error,processed;
static uint8_t request[8];
enum { NORMAL,MISSING,PARTIAL,DROP_FIRST,WRONG_ID,CHECKSUM,OVERRUN,FRAMING,NOISE,LATE,DUPLICATE,ECHO,TX_STUCK };
static int mode;
static unsigned fail_request;
static uint32_t command_times[256];static unsigned commands;
typedef struct { uint64_t at;uint8_t byte;uint32_t error;bool used; } Event;
static Event events[256];
static unsigned event_count;

uint32_t servo_test_cycles(void){return clock_base+(uint32_t)(us*84);}
uint32_t servo_test_lock(void){uint32_t p=masked;masked=1;return p;}
void servo_test_unlock(uint32_t p){masked=p;}
uint32_t servo_test_read_dr(USART_TypeDef *u){uint32_t b=u->DR;u->SR&=~(USART_SR_RXNE|15U);return b;}
uint32_t HAL_GetTick(void){return (uint32_t)(us/1000);}

static void event(uint64_t at,uint8_t byte,uint32_t error)
{
    /* Reuse consumed slots; no unbounded allocation hides RX overflow. */
    unsigned i=0;while(i<event_count && !events[i].used)++i;
    if(i==event_count){assert(event_count<256);++event_count;}
    events[i]=(Event){at,byte,error,false};
}
static void reply(void)
{
    ++requests;
    int m=(!fail_request || requests==fail_request)?mode:NORMAL;
    if(m==MISSING || m==TX_STUCK)return;
    uint8_t r[21]={255,255,request[2],17,0};
    r[5]=0x00;r[6]=0x08;r[11]=110;r[12]=30;
    if(m==WRONG_ID)++r[2];
    unsigned sum=0;for(unsigned i=2;i<20;++i)sum+=r[i];r[20]=(uint8_t)~sum;
    if(m==CHECKSUM)r[20]^=1;
    unsigned end=m==PARTIAL?9:21,start=m==DROP_FIRST?1:0;
    uint64_t begin=us+(m==LATE?7000:70);
    for(unsigned i=start;i<end;++i) {
        uint32_t error=i==6?(m==OVERRUN?USART_SR_ORE:m==FRAMING?USART_SR_FE:m==NOISE?USART_SR_NE:0):0;
        event(begin+i*10,r[i],error);
    }
    if(m==DUPLICATE)for(unsigned i=0;i<21;++i)event(begin+500+i*10,r[i],0);
}
void servo_test_write_dr(USART_TypeDef *u,uint8_t byte)
{
    u->DR=byte;u->SR&=~(USART_SR_TXE|USART_SR_TC);
    ++tx_bytes;request[bus.feedback.tx_index-1]=byte;
    if(mode==ECHO)event(us+10,byte,0);
    if(bus.feedback.tx_index==8)reply();
}
static void pump(unsigned duration)
{
    uint64_t end=us+duration;
    while(us<end) {
        us+=10;
        if(mode!=TX_STUCK)regs.SR|=USART_SR_TXE|USART_SR_TC;
        for(unsigned i=0;i<event_count;++i)if(!events[i].used && events[i].at<=us) {
            if(regs.SR&USART_SR_RXNE)regs.SR|=USART_SR_ORE;
            regs.DR=events[i].byte;regs.SR|=USART_SR_RXNE|events[i].error;events[i].used=true;
        }
        if(!masked && (((regs.CR1&USART_CR1_RXNEIE) && (regs.SR&(USART_SR_RXNE|15U))) ||
           ((regs.CR1&USART_CR1_TXEIE) && (regs.SR&USART_SR_TXE)) ||
           ((regs.CR1&USART_CR1_TCIE) && (regs.SR&USART_SR_TC))))servo_bus_feedback_irq();
        if(!masked && us%1000==0)servo_bus_feedback_tick();
    }
}
void HAL_Delay(uint32_t ms){pump((ms+1)*1000);}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *u,uint8_t *bytes,uint16_t n,uint32_t timeout)
{
    assert(!masked);assert(n>=6);
    if(bus.feedback.enabled) {
        assert(timeout==1);assert(bytes[2]==254);assert(commands<256);
        command_times[commands++]=(uint32_t)us;
    }
    pump(n*10);
    if(legacy_error){u->Instance->DR=255;u->Instance->SR|=USART_SR_RXNE|legacy_error;}
    return HAL_OK;
}
static void setup(int m)
{
    us=0;clock_base=masked=requests=tx_bytes=legacy_error=processed=commands=event_count=fail_request=0;
    memset(&regs,0,sizeof regs);regs.SR=USART_SR_TXE|USART_SR_TC;
    memset(&bus,0,sizeof bus);servo_bus_init(&bus,&uart,25);mode=m;
    servo_bus_feedback_begin(&bus);
}
static ServoBusResult take(uint8_t data[15])
{
    uint32_t begin,end;uint64_t before=us;
    ServoBusResult r=servo_bus_feedback_take(&bus,data,&begin,&end);
    assert(us==before); /* Nonblocking, including missing/partial responses. */
    return r;
}
static void transport_cases(void)
{
    for(int m=NORMAL;m<=TX_STUCK;++m) {
        setup(m);uint8_t data[15];memset(data,0x55,15);
        uint64_t before=us;
        assert(servo_bus_feedback_start(&bus,9,56,15,0)==SERVO_BUS_OK && us==before);
        assert(!servo_bus_feedback_can_write(&bus));
        assert(take(data)==SERVO_BUS_BUSY);
        pump(10000);ServoBusResult r=take(data);
        if(m==NORMAL || m==DUPLICATE || m==ECHO) {
            assert(r==SERVO_BUS_OK && data[0]==0 && data[1]==8);
            assert(bus.feedback.ok==1 && bus.feedback.timeouts==0);
        } else {
            assert(r!=SERVO_BUS_OK && r!=SERVO_BUS_BUSY);
            for(unsigned i=0;i<15;++i)assert(data[i]==0x55);
            assert(servo_bus_feedback_start(&bus,9,56,15,1)==SERVO_BUS_BUSY);
            if(m==OVERRUN || m==FRAMING || m==NOISE)assert(r==SERVO_BUS_HAL_ERROR && bus.feedback.trace[0].uart_errors);
            if(m==MISSING || m==PARTIAL || m==LATE || m==TX_STUCK)assert(r==SERVO_BUS_TIMEOUT);
            pump(30000);mode=NORMAL;
            assert(servo_bus_feedback_start(&bus,9,56,15,1)==SERVO_BUS_OK);
            pump(8000);assert(take(data)==SERVO_BUS_OK);
        }
        assert(take(data)==SERVO_BUS_BUSY); /* consume once */
        if(m==DUPLICATE || m==LATE)assert(bus.feedback.late_bytes>0);
        servo_bus_feedback_end(&bus);
        assert(!(regs.CR1&(USART_CR1_RXNEIE|USART_CR1_TXEIE|USART_CR1_TCIE)));
    }
    puts("13 UART scenarios: normal/missing/partial/header/ID/checksum/ORE/FE/NE/late/duplicate/echo/TX stall passed");
}

static RobotResult bus_failure(RobotController *r,uint8_t id,ServoBusResult result)
{r->last_failed_servo_id=id;r->last_bus_result=result;return ROBOT_BUS_ERROR;}
static RobotResult process_joint_sample(RobotController *r,size_t index,const uint16_t targets[12],
    uint16_t phase,uint16_t *position,bool *tripped,Sts3215State state,ServoBusResult result,uint32_t begin,uint32_t end)
{
    (void)r;(void)index;(void)targets;(void)phase;(void)position;(void)tripped;
    assert(result==SERVO_BUS_OK && state.position==2048 && state.voltage_mv==11000);
    assert(end-begin<=5);++processed;return ROBOT_OK;
}
/* Generated from robot.c by the Python runner, not a copied retry policy. */
#include "gait_feedback_under_test.inc"

static void gait_cases(void)
{
    setup(MISSING);fail_request=5;
    RobotController robot={.bus=&bus};GaitFeedback f={.attempt_frame=UINT32_MAX};
    uint16_t targets[12];for(unsigned j=0;j<12;++j)targets[j]=2048;
    for(unsigned frame=0;frame<80;++frame) {
        uint64_t start=us;
        assert(gait_feedback_take(&robot,&f,targets,0)==ROBOT_OK);
        assert(gait_feedback_fresh(&robot,&f,HAL_GetTick())==ROBOT_OK);
        pump(12000); /* foreground computation; UART and SysTick keep running */
        assert(gait_feedback_take(&robot,&f,targets,0)==ROBOT_OK);
        assert(sts3215_sync_positions(&bus,g_robot_servo_ids,targets,12)==SERVO_BUS_OK);
        gait_feedback_start(&robot,&f,frame);
        pump((unsigned)(start+20000-us));
    }
    assert(commands==80 && robot.bus->read_retry_attempts==1 && robot.bus->read_retry_recoveries==1);
    assert(robot.bus->read_retry_failures==0 && processed>70);
    for(unsigned i=1;i<commands;++i)assert(command_times[i]-command_times[i-1]==20000);
    printf("Missing reply + deferred retry: %u writes, max gap 20000 us, recovered once\n",commands);
    assert(gait_feedback_take(&robot,&f,targets,0)==ROBOT_OK);
    /* Repeated failure stops; it must not become indefinite blind motion. */
    servo_bus_feedback_end(&bus);setup(MISSING);memset(&robot,0,sizeof robot);robot.bus=&bus;
    f=(GaitFeedback){.attempt_frame=UINT32_MAX};
    gait_feedback_start(&robot,&f,0);pump(10000);
    assert(gait_feedback_take(&robot,&f,targets,0)==ROBOT_OK && f.retry);
    gait_feedback_start(&robot,&f,0);assert(!f.waiting); /* no immediate retry */
    pump(30000);gait_feedback_start(&robot,&f,2);pump(10000);
    assert(gait_feedback_take(&robot,&f,targets,0)==ROBOT_BUS_ERROR);
    assert(bus.read_retry_failures==1);
    pump(500000);assert(gait_feedback_fresh(&robot,&f,HAL_GetTick())==ROBOT_BUS_ERROR);
    servo_bus_feedback_end(&bus);
    puts("Production gait retry and stale-feedback stop policy passed");

    setup(NORMAL);memset(&robot,0,sizeof robot);robot.bus=&bus;
    f=(GaitFeedback){.attempt_frame=UINT32_MAX};
    for(unsigned frame=0;frame<40;++frame) {
        uint64_t start=us;
        assert(gait_feedback_take(&robot,&f,targets,0)==ROBOT_OK);
        if(frame && !f.retry)gait_feedback_start(&robot,&f,frame);
        pump(12000);
        assert(gait_feedback_take(&robot,&f,targets,0)==ROBOT_OK);
        assert(sts3215_sync_positions(&bus,g_robot_servo_ids,targets,12)==SERVO_BUS_OK);
        gait_feedback_start(&robot,&f,frame);
        pump((unsigned)(start+20000-us));
    }
    assert(gait_feedback_take(&robot,&f,targets,0)==ROBOT_OK);
    assert(requests==79 && processed==79);
    for(unsigned i=1;i<commands;++i)assert(command_times[i]-command_times[i-1]==20000);
    servo_bus_feedback_end(&bus);
    puts("V625 two feedback slots: 79 reads / 40 frames, 20 ms command spacing passed");
}

static void boundary_cases(void)
{
    setup(NORMAL);clock_base=UINT32_MAX-84000U;
    uint8_t data[15];assert(servo_bus_feedback_start(&bus,9,56,15,0)==SERVO_BUS_OK);
    pump(8000);assert(take(data)==SERVO_BUS_OK && bus.feedback.max_us<1000);
    servo_bus_feedback_end(&bus);
    setup(MISSING);assert(servo_bus_feedback_start(&bus,9,56,15,0)==SERVO_BUS_OK);
    pump(2000);uint64_t at=us;servo_bus_feedback_end(&bus);assert(us==at);
    pump(8000);assert(!bus.feedback.enabled); /* STOP cleanup does not await a reply. */
    /* Legacy RXNE+error must not be accepted as ordinary data. */
    servo_bus_feedback_legacy_guard(&bus);legacy_error=USART_SR_ORE;
    uint8_t params[2]={56,15};size_t size;
    assert(servo_bus_request(&bus,9,2,params,2,data,15,&size,true)==SERVO_BUS_HAL_ERROR);
    puts("DWT wrap, nonblocking cancellation, legacy RXNE+ORE passed");
}

static void diagnostic_cases(void)
{
    setup(NORMAL);uint8_t data[15];
    for(unsigned i=0;i<40;++i) {
        assert(servo_bus_feedback_start(&bus,9,56,15,0)==SERVO_BUS_OK);
        pump(8000);assert(take(data)==SERVO_BUS_OK);
    }
    assert(bus.feedback.trace_count==32 && !bus.feedback.trace_frozen);
    mode=OVERRUN;
    assert(servo_bus_feedback_start(&bus,9,56,15,0)==SERVO_BUS_OK);
    pump(8000);assert(take(data)==SERVO_BUS_HAL_ERROR);
    unsigned failure=(bus.feedback.trace_write+31)%32;
    const ServoReadTrace *t=&bus.feedback.trace[failure];
    assert(t->id==9 && t->attempt==0 && t->result==SERVO_BUS_HAL_ERROR);
    assert(t->stage==3 && t->sent==8 && t->uart_errors==USART_SR_ORE);
    assert(t->received==6 && t->first_rx_us<=t->last_rx_us && t->last_rx_us<=t->duration_us);
    pump(30000);mode=NORMAL;
    for(unsigned i=0;i<8;++i) {
        assert(servo_bus_feedback_start(&bus,9,56,15,i?0:1)==SERVO_BUS_OK);
        pump(8000);assert(take(data)==SERVO_BUS_OK);
    }
    assert(bus.feedback.trace_frozen && bus.feedback.trace_triggered);
    ServoReadTrace saved[32];memcpy(saved,bus.feedback.trace,sizeof saved);
    unsigned writes=bus.feedback.trace_write;
    assert(servo_bus_feedback_start(&bus,10,56,15,0)==SERVO_BUS_OK);
    pump(8000);assert(take(data)==SERVO_BUS_OK);
    assert(bus.feedback.trace_write==writes && !memcmp(saved,bus.feedback.trace,sizeof saved));
    assert(bus.feedback.ok==49 && bus.feedback.errors==1);
    servo_bus_feedback_end(&bus);
    puts("Diagnostic ring rollover, error metadata and first-error freeze passed");
}
int main(void){transport_cases();gait_cases();boundary_cases();diagnostic_cases();return 0;}

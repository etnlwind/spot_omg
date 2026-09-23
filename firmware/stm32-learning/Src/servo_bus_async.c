#include "servo_bus.h"
#include "feetech_protocol.h"
#include <string.h>

enum { IDLE, QUEUED, TX, RX, DONE };
#define RX_ERRORS (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)
static ServoBus * volatile active_bus;

/* Only these tiny hardware boundaries are substituted by the host UART model. */
#ifndef SERVO_BUS_HOST_TEST
static uint32_t cycles(void) { return DWT->CYCCNT; }
static uint32_t lock(void) { uint32_t p=__get_PRIMASK(); __disable_irq(); return p; }
static void unlock(uint32_t p) { __set_PRIMASK(p); }
static uint32_t read_dr(USART_TypeDef *u) { return u->DR; }
static void write_dr(USART_TypeDef *u,uint8_t b) { u->DR=b; }
#else
extern uint32_t servo_test_cycles(void),servo_test_lock(void);
extern void servo_test_unlock(uint32_t),servo_test_write_dr(USART_TypeDef *,uint8_t);
extern uint32_t servo_test_read_dr(USART_TypeDef *);
#define cycles servo_test_cycles
#define lock servo_test_lock
#define unlock servo_test_unlock
#define read_dr servo_test_read_dr
#define write_dr servo_test_write_dr
#endif

static uint32_t elapsed(const ServoFeedback *f,uint32_t now,uint32_t since)
{ return (uint32_t)(now-since)/f->cycles_per_us; }

static void finish(ServoBus *bus,ServoBusResult result,uint32_t now)
{
    ServoFeedback *f=&bus->feedback;
    CLEAR_BIT(bus->uart->Instance->CR1,USART_CR1_TXEIE|USART_CR1_TCIE);
    f->result=result; f->end_ms=HAL_GetTick();
    uint32_t duration=elapsed(f,now,f->started);
    if(duration>f->max_us)f->max_us=duration;
    if(result==SERVO_BUS_OK)++f->ok;
    else {
        if(result==SERVO_BUS_TIMEOUT)++f->timeouts;else ++f->errors;
        /* Do not mistake a delayed reply for the next read of this ID. */
        f->quarantined=true;f->quarantine=now;
    }
    if(!f->trace_frozen) {
        ServoReadTrace *t=&f->trace[f->trace_write];
        *t=(ServoReadTrace){f->start_ms,duration,
            f->rx_count?elapsed(f,f->first_rx,f->started):UINT32_MAX,
            f->rx_count?elapsed(f,f->last_rx,f->started):UINT32_MAX,
            f->uart_errors,f->id,f->attempt,(uint8_t)result,f->rx_count,f->state,f->tx_index};
        f->trace_write=(f->trace_write+1U)%SERVO_FEEDBACK_TRACE_CAPACITY;
        if(f->trace_count<SERVO_FEEDBACK_TRACE_CAPACITY)++f->trace_count;
        if(result!=SERVO_BUS_OK && !f->trace_triggered) {
            f->trace_triggered=true;f->trace_tail=8;
        } else if(f->trace_triggered && f->trace_tail && --f->trace_tail==0)
            f->trace_frozen=true;
    }
    __DMB(); f->state=DONE;
}

static void parse_ready(ServoBus *bus)
{
    ServoFeedback *f=&bus->feedback;
    if(f->state!=RX)return;
    uint8_t count=f->rx_count,offset=0;
    if(count<8 && !memcmp(f->rx,f->packet,count))return;
    if(count>=8 && !memcmp(f->rx,f->packet,8))offset=8;
    uint8_t n=count-offset;
    if(n<4)return;
    const uint8_t *r=f->rx+offset;
    if(r[0]!=255 || r[1]!=255 || r[2]!=f->id || r[3]!=f->size+2) {
        finish(bus,SERVO_BUS_PROTOCOL_ERROR,cycles());return;
    }
    if(n<f->size+6)return;
    ServoBusResult result=SERVO_BUS_OK;
    if(n!=f->size+6 || !feetech_packet_checksum_valid(r,n))result=SERVO_BUS_PROTOCOL_ERROR;
    else if(r[4]) {bus->last_servo_error=r[4];result=SERVO_BUS_SERVO_ERROR;}
    if(result==SERVO_BUS_OK)memcpy(f->payload,r+5,f->size);
    finish(bus,result,f->last_rx);
}

void servo_bus_feedback_begin(ServoBus *bus)
{
    uint32_t p=lock();
    memset(&bus->feedback,0,sizeof(bus->feedback));
    ServoFeedback *f=&bus->feedback;
#ifndef SERVO_BUS_HOST_TEST
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
#endif
    f->cycles_per_us=SystemCoreClock/1000000U;
    f->last_activity=cycles();f->enabled=true;active_bus=bus;
    (void)bus->uart->Instance->SR;(void)read_dr(bus->uart->Instance);
    SET_BIT(bus->uart->Instance->CR1,USART_CR1_RE|USART_CR1_RXNEIE);
    SET_BIT(bus->uart->Instance->CR3,USART_CR3_EIE);
    unlock(p);
}

void servo_bus_feedback_end(ServoBus *bus)
{
    uint32_t p=lock();
    ServoFeedback *f=&bus->feedback;
    if(f->enabled && (f->state==QUEUED || f->state==TX || f->state==RX))
        finish(bus,SERVO_BUS_TIMEOUT,cycles());
    CLEAR_BIT(bus->uart->Instance->CR1,USART_CR1_RXNEIE|USART_CR1_TXEIE|USART_CR1_TCIE);
    CLEAR_BIT(bus->uart->Instance->CR3,USART_CR3_EIE);
    f->enabled=false;f->state=IDLE;
    if(active_bus==bus)active_bus=NULL;
    unlock(p);
}

ServoBusResult servo_bus_feedback_start(ServoBus *bus,uint8_t id,
                                       uint8_t address,uint8_t size,uint8_t attempt)
{
    if(!bus || id>=FEETECH_BROADCAST_ID || !size || size>15)return SERVO_BUS_INVALID_ARGUMENT;
    uint32_t p=lock();ServoFeedback *f=&bus->feedback;uint32_t now=cycles();
    if(!f->enabled || f->state!=IDLE ||
       (f->quarantined && elapsed(f,now,f->quarantine)<SERVO_FEEDBACK_QUARANTINE_US)) {
        unlock(p);return SERVO_BUS_BUSY;
    }
    f->quarantined=false;f->id=id;f->size=size;f->attempt=attempt;
    bus->last_servo_error=0;
    uint8_t params[2]={address,size};
    (void)feetech_encode_instruction(id,FEETECH_INST_READ,params,2,f->packet,8);
    f->tx_index=f->rx_count=0;f->uart_errors=0;f->queued=now;
    f->started=now;f->start_ms=HAL_GetTick();f->state=QUEUED;
    unlock(p);return SERVO_BUS_OK;
}

/* SysTick performs only bounded bookkeeping. UART IRQ owns byte transfer.
 * No parsing of console commands, HAL waits or formatting happens here. */
void servo_bus_feedback_tick(void)
{
    ServoBus *bus=active_bus;if(!bus)return;
    uint32_t p=lock();ServoFeedback *f=&bus->feedback;uint32_t now=cycles();
    if(f->state==QUEUED) {
        if(elapsed(f,now,f->queued)>=SERVO_FEEDBACK_WINDOW_US)
            finish(bus,SERVO_BUS_TIMEOUT,now);
        else if(elapsed(f,now,f->last_activity)>=SERVO_FEEDBACK_QUIET_US) {
            f->started=now;f->start_ms=HAL_GetTick();f->state=TX;
            SET_BIT(bus->uart->Instance->CR1,USART_CR1_TXEIE);
        }
    } else if((f->state==TX || f->state==RX) &&
              elapsed(f,now,f->started)>=SERVO_FEEDBACK_WINDOW_US)
        finish(bus,SERVO_BUS_TIMEOUT,now);
    unlock(p);
}

void servo_bus_feedback_irq(void)
{
    ServoBus *bus=active_bus;if(!bus)return;
    ServoFeedback *f=&bus->feedback;USART_TypeDef *u=bus->uart->Instance;
    uint32_t sr=u->SR,now=cycles();
    if(sr&(USART_SR_RXNE|RX_ERRORS)) {
        uint8_t b=(uint8_t)read_dr(u); /* SR was captured BEFORE DR clears errors. */
        f->last_activity=now;
        if(f->state==TX || f->state==RX) {
            f->uart_errors|=sr&RX_ERRORS;
            if(sr&RX_ERRORS)finish(bus,SERVO_BUS_HAL_ERROR,now);
            else if(elapsed(f,now,f->started)>=SERVO_FEEDBACK_WINDOW_US) {
                ++f->late_bytes;finish(bus,SERVO_BUS_TIMEOUT,now);
            } else if(f->rx_count>=sizeof(f->rx))finish(bus,SERVO_BUS_PROTOCOL_ERROR,now);
            else {
                if(!f->rx_count)f->first_rx=now;
                f->last_rx=now;f->rx[f->rx_count++]=b;
            }
        } else if(!f->writing)++f->late_bytes;
    }
    /* TX/RX remain enabled together: an adapter echo is filtered as a packet,
     * without a blind RE-off interval at the beginning of the servo reply. */
    if(f->state==TX && (u->CR1&USART_CR1_TXEIE) && (sr&USART_SR_TXE)) {
        write_dr(u,f->packet[f->tx_index++]);
        if(f->tx_index==sizeof(f->packet)) {
            CLEAR_BIT(u->CR1,USART_CR1_TXEIE);SET_BIT(u->CR1,USART_CR1_TCIE);
        }
    } else if(f->state==TX && (u->CR1&USART_CR1_TCIE) && (sr&USART_SR_TC)) {
        CLEAR_BIT(u->CR1,USART_CR1_TCIE);f->state=RX;
    }
    /* A complete reply must finish even while foreground IMU/math is busy. */
    parse_ready(bus);
    uint32_t irq_cycles=(uint32_t)(cycles()-now);
    if(irq_cycles>f->irq_peak_cycles)f->irq_peak_cycles=irq_cycles;
}

/* Consume the ISR result; never wait for bytes or run a retry here. */
ServoBusResult servo_bus_feedback_take(ServoBus *bus,uint8_t *data,
                                      uint32_t *begin_ms,uint32_t *end_ms)
{
    ServoFeedback *f=&bus->feedback;
    uint32_t p=lock();
    if(f->state!=DONE) {unlock(p);return SERVO_BUS_BUSY;}
    ServoBusResult result=f->result;
    if(result==SERVO_BUS_OK)memcpy(data,f->payload,f->size);
    *begin_ms=f->start_ms;*end_ms=f->end_ms;
    f->state=IDLE;unlock(p);return result;
}

bool servo_bus_feedback_can_write(ServoBus *bus)
{
    ServoFeedback *f=&bus->feedback;
    if(!f->enabled)return true;
    uint32_t p=lock();
    bool ok=f->state==IDLE && elapsed(f,cycles(),f->last_activity)>=SERVO_FEEDBACK_QUIET_US;
    if(!ok)++f->refused_writes;else f->writing=true;
    unlock(p);return ok;
}
void servo_bus_feedback_written(ServoBus *bus)
{
    uint32_t p=lock();bus->feedback.last_activity=cycles();bus->feedback.writing=false;unlock(p);
}

void servo_bus_feedback_legacy_guard(ServoBus *bus)
{
    ServoFeedback *f=&bus->feedback;
    /* Used only AFTER walking ends, before a synchronous pose/config read. */
    if(f->enabled || !f->quarantined)return;
    while(elapsed(f,cycles(),f->quarantine)<SERVO_FEEDBACK_QUARANTINE_US)HAL_Delay(1);
    f->quarantined=false;
}

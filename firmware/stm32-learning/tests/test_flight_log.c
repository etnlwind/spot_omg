#include "flight_log.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned char flash[128*1024];
static uint32_t tick;
static int budget=-1;
TestRcc test_rcc={0};
const void *flight_log_test_pointer(uint32_t address) {
    assert(address>=0x08060000 && address<0x08080000);
    return flash+(address-0x08060000);
}
uint32_t HAL_GetTick(void){return tick++;}
int HAL_FLASH_Unlock(void){return HAL_OK;}
int HAL_FLASH_Lock(void){return HAL_OK;}
int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase,uint32_t *error) {
    assert(erase->Sector==7);*error=0;memset(flash,0xff,sizeof(flash));return HAL_OK;
}
int HAL_FLASH_Program(uint32_t type,uint32_t address,uint64_t value) {
    (void)type;
    if(budget==0)return HAL_ERROR;
    if(budget>0)budget--;
    unsigned char *ptr=flash+(address-0x08060000);
    uint32_t word=(uint32_t)value,previous;
    memcpy(&previous,ptr,4);
    assert((previous&word)==word);
    memcpy(ptr,&word,4);return HAL_OK;
}
static bool contains(const char *text) {
    FlightLogEntry entry;
    for(size_t i=0;i<flight_log_count();i++) {
        assert(flight_log_get(i,&entry));
        if(!strcmp(entry.text,text))return true;
    }
    return false;
}
int main(void) {
    memset(flash,0xff,sizeof(flash));memset(flash,0xa5,1024);
    flight_log_init("test");assert(flight_log_append("before-reset"));
    size_t before=flight_log_count();
    budget=5;assert(!flight_log_append("interrupted"));budget=-1;
    flight_log_init("test");
    assert(flight_log_count()==before+1);
    assert(contains("before-reset") && !contains("interrupted"));
    assert(flight_log_append("after-reset") && contains("after-reset"));
    budget=0;assert(!flight_log_append("erased-gap"));budget=-1;
    assert(flight_log_append("after-gap"));
    flight_log_init("test");
    assert(contains("before-reset") && contains("after-reset") && contains("after-gap"));
    assert(!flight_log_prepare_entries(0) && !flight_log_prepare_entries(1017));
    while(flight_log_count()<990)assert(flight_log_append("fill"));
    assert(flight_log_prepare_entries(50));assert(flight_log_count()==0);
    for(unsigned i=0;i<1024;i++)assert(flash[i]==0xa5);
    for(unsigned i=0;i<50;i++)assert(flight_log_append("batch"));
    assert(flight_log_count()==50);
    flight_log_init("test");assert(flight_log_count()==51);
    assert(flight_log_clear());assert(flight_log_count()==0);
    for(unsigned i=0;i<1024;i++)assert(flash[i]==0xa5);
    puts("flight log: partial reset recovery, erased gaps, batch rotation and calibration preservation passed");
}

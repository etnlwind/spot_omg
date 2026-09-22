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
    uint32_t values[4]={30,40,5,6}, loaded[4]={0};
    assert(!flight_log_load_foot_lift(loaded));
    assert(flight_log_save_foot_lift(values));
    assert(flight_log_load_foot_lift(loaded) && !memcmp(values,loaded,sizeof(values)));
    before=flight_log_count();assert(flight_log_save_foot_lift(values));assert(flight_log_count()==before);
    int32_t widths[4]={-5,6,-7,8}, width_read[4];
    assert(flight_log_load_foot_width(width_read));
    for(int i=0;i<4;i++)assert(width_read[i]==0);
    assert(flight_log_save_foot_settings(values,widths));
    flight_log_init("width-reboot");
    assert(flight_log_load_foot_width(width_read) && !memcmp(widths,width_read,sizeof widths));
    /* Legacy clients updating lift must not clear width. */
    assert(flight_log_save_foot_lift(values));
    assert(flight_log_load_foot_width(width_read) && !memcmp(widths,width_read,sizeof widths));
    /* Every word-boundary interruption keeps the last committed settings. */
    for (int cut=0;cut<32;cut++) {
        uint32_t changed[4]={70,80,9,10};
        int32_t changed_widths[4]={9,-10,11,-12};
        budget=cut;assert(!flight_log_save_foot_settings(changed,changed_widths));budget=-1;
        flight_log_init("restart");
        assert(flight_log_load_foot_width(width_read) && !memcmp(widths,width_read,sizeof widths));
        assert(flight_log_load_foot_lift(loaded) && !memcmp(values,loaded,sizeof(values)));
    }
    assert(flight_log_clear());
    flight_log_init("after-clear");
    assert(flight_log_load_foot_lift(loaded) && !memcmp(values,loaded,sizeof(values)));
    for(unsigned i=0;i<256;i++)assert(flash[i]==0xa5);
    unsigned char calibration[44];memset(calibration,0x19,sizeof(calibration));
    assert(flight_log_save_calibration(calibration,sizeof(calibration)));
    assert(!memcmp(flash,calibration,sizeof(calibration)));
    flight_log_init("after-imu-save");
    assert(flight_log_load_foot_lift(loaded) && !memcmp(values,loaded,sizeof(values)));
    while(flight_log_count()<1000)assert(flight_log_append("fill"));
    assert(flight_log_prepare_entries(50));
    assert(flight_log_load_foot_lift(loaded) && !memcmp(values,loaded,sizeof(values)));
    uint32_t invalid[4]={0,0,0,0x80000000};assert(!flight_log_save_foot_lift(invalid));
    assert(flight_log_load_foot_width(width_read) && !memcmp(widths,width_read,sizeof widths));
    puts("flight log: partial reset recovery, erased gaps, batch rotation and calibration preservation passed");
}

#include "sts3215.h"
#include "feetech_protocol.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static uint8_t registers[12][80];
static unsigned calls;static bool fail_first;
ServoBusResult servo_bus_ping(ServoBus*b,uint8_t id){(void)b;(void)id;return SERVO_BUS_OK;}
ServoBusResult servo_bus_read(ServoBus*b,uint8_t id,uint8_t a,uint8_t*d,size_t n){(void)b;memcpy(d,&registers[id-1][a],n);return SERVO_BUS_OK;}
ServoBusResult servo_bus_write(ServoBus*b,uint8_t id,uint8_t a,const uint8_t*d,size_t n){(void)b;memcpy(&registers[id-1][a],d,n);return SERVO_BUS_OK;}
ServoBusResult servo_bus_sync_write(ServoBus*b,uint8_t a,uint8_t size,const uint8_t*ids,const uint8_t*items,size_t n){
 (void)b;calls++;if(fail_first)return SERVO_BUS_TIMEOUT;
 for(unsigned i=0;i<n;i++)memcpy(&registers[ids[i]-1][a],items+i*size,size);
 return SERVO_BUS_OK;
}
int main(void){
 ServoBus bus={0};uint8_t ids[12];uint16_t positions[12];
 for(unsigned i=0;i<12;i++){ids[i]=i+1;positions[i]=2000+i;registers[i][40]=1;}
 assert(sts3215_sync_move(&bus,ids,positions,12,300,30)==SERVO_BUS_OK);
 for(unsigned i=0;i<12;i++)positions[i]+=5;
 assert(sts3215_sync_positions(&bus,ids,positions,12)==SERVO_BUS_OK);
 for(unsigned i=0;i<12;i++)assert(registers[i][41]==30 && feetech_decode_u16(registers[i]+46)==300);
 uint8_t before[12][80];memcpy(before,registers,sizeof before);
 /* Signed goals and nonzero time must be preserved byte-for-byte. */
 for(unsigned i=0;i<12;i++){
  registers[i][43]|=0x80;registers[i][44]=17;registers[i][45]=3;
 }
 memcpy(before,registers,sizeof before);
 assert(sts3215_sync_profile(&bus,ids,12,3400,254)==SERVO_BUS_OK);
 for(unsigned i=0;i<12;i++)for(unsigned a=0;a<80;a++){
  if(a==41)assert(registers[i][a]==254);
  else if(a==46 || a==47)assert(feetech_decode_u16(registers[i]+46)==3400);
  else assert(registers[i][a]==before[i][a]);
 }
 /* Position-only gait updates must retain the maximum profile on all axes. */
 assert(sts3215_sync_positions(&bus,ids,positions,12)==SERVO_BUS_OK);
 for(unsigned i=0;i<12;i++)assert(registers[i][41]==254);
 /* Explicit lower settings remain usable for controlled pose transitions. */
 assert(sts3215_sync_profile(&bus,ids,12,3400,100)==SERVO_BUS_OK);
 for(unsigned i=0;i<12;i++)assert(registers[i][41]==100);
 assert(sts3215_sync_profile(&bus,ids,12,3400,255)==SERVO_BUS_INVALID_ARGUMENT);
 calls=0;fail_first=true;
 assert(sts3215_sync_profile(&bus,ids,12,3400,254)==SERVO_BUS_TIMEOUT && calls==1);
 assert(sts3215_sync_profile(&bus,ids,13,3400,254)==SERVO_BUS_INVALID_ARGUMENT && calls==1);
 puts("persistent low-speed reproduction, profile-only restore, unchanged goals/torque/origins and write failure passed");
}

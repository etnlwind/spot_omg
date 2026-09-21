#undef NDEBUG
#define BNO055_H
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
typedef enum {HAL_OK,HAL_ERROR} HAL_StatusTypeDef;
typedef struct {bool stuck;} I2C_HandleTypeDef;
typedef struct {bool valid;} BodyImuState;
typedef struct {
 I2C_HandleTypeDef *i2c;bool present,cached_yaw_valid,body_metadata_valid;
 BodyImuState cached_body_imu;uint16_t address;int calibration;
} Bno055;
#define I2C_MEMADD_SIZE_8BIT 1
static int failure,calls,reads,delay_ms,body_reads,attitude_reads;
static HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef*b){(void)b;calls++;return failure==1?HAL_ERROR:HAL_OK;}
static HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef*b){calls++;b->stuck=false;return failure==2?HAL_ERROR:HAL_OK;}
static HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef*b,uint16_t a,uint16_t r,uint16_t m,uint8_t*v,uint16_t n,uint32_t t){
 assert(!b->stuck && a==0x52 && m==1 && n==1 && t==20);reads++;
 *v=r==0?0xa0:8;if(failure==4)*v=0;return failure==3?HAL_ERROR:HAL_OK;
}
static bool bno055_read_body_imu(void*p,BodyImuState*s){Bno055*i=p;assert(!i->cached_body_imu.valid);body_reads++;s->valid=true;return failure!=5;}
static bool bno055_read_attitude(void*p,int16_t*r,int16_t*t){(void)p;*r=*t=0;attitude_reads++;return failure!=6;}
static void HAL_Delay(uint32_t t){delay_ms+=(int)t;}
#include "imu_bus_recovery.h"
int main(void){
 for(failure=0;failure<=6;failure++) {
  I2C_HandleTypeDef bus={true};Bno055 imu={.i2c=&bus,.present=true,.address=0x52,.calibration=123,
   .cached_yaw_valid=true,.body_metadata_valid=true,.cached_body_imu={true}};
  ImuBusRecovery result;calls=reads=delay_ms=body_reads=attitude_reads=0;
  assert(!imu_bus_recover(&imu,true,&result) && calls==0 && imu.cached_body_imu.valid);
  assert(imu_bus_recover(&imu,false,&result)==(failure==0));
  assert(imu.calibration==123 && imu.address==0x52 && imu.present);
  if(!failure)assert(result.samples==3 && body_reads==3 && attitude_reads==3 && delay_ms==40);
  else assert(result.samples==0);
  if(failure==1)assert(calls==1 && reads==0);
  if(failure==2)assert(calls==2 && reads==0);
 }
 ImuBusRecovery result;calls=0;assert(!imu_bus_recover(NULL,false,&result) && !calls);
 puts("IMU bus recovery guard/failure/success tests passed");return 0;
}

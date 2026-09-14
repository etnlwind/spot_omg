#include "bno055.h"
#include "bno_imu_config.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t tick,reads,writes,latency;
static uint16_t fail_reg;
static uint8_t metadata[8],payload[12];
static bool legacy_read;
static void closef(float got,float want) { assert(fabsf(got-want)<2e-6f); }
static void put16(uint8_t *bytes,int value) {
    const unsigned raw=(unsigned)value&65535U;
    bytes[0]=(uint8_t)raw; bytes[1]=(uint8_t)(raw>>8);
}
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick+=ms; }
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *i2c) { return i2c->state; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c,uint16_t address,
    uint16_t reg,uint16_t size_type,uint8_t *bytes,uint16_t size,uint32_t timeout) {
    assert(i2c && address==0x50 && size_type==I2C_MEMADD_SIZE_8BIT);
    ++reads; tick+=latency;
    assert(timeout==(legacy_read ? 100U : BNO055_BODY_READ_TIMEOUT_MS));
    if (reg==fail_reg) { tick+=timeout+1U; return HAL_TIMEOUT; }
    if (reg==0x3b) { assert(size==8); memcpy(bytes,metadata,8); }
    else if (reg==0x14) { assert(size==12); memcpy(bytes,payload,12); }
    else if (reg==0x1a) { assert(legacy_read && size==6); memcpy(bytes,payload+6,6); }
    else assert(0);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c,uint16_t a,uint16_t r,
    uint16_t mt,uint8_t *b,uint16_t n,uint32_t t) {
    (void)i2c;(void)a;(void)r;(void)mt;(void)b;(void)n;(void)t;++writes;return HAL_ERROR;
}
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *i,uint16_t a,uint32_t n,uint32_t t) {
    (void)i;(void)a;(void)n;(void)t;return HAL_ERROR;
}
void HAL_FLASH_Unlock(void) { assert(0); }
void HAL_FLASH_Lock(void) { assert(0); }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,uint32_t *s) {
    (void)e;(void)s;assert(0);return HAL_ERROR;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t t,uint32_t a,uint64_t v) {
    (void)t;(void)a;(void)v;assert(0);return HAL_ERROR;
}
void flight_log_on_sector_reformatted(void) { assert(0); }

static Bno055BodyFrame identity(void) {
    Bno055BodyFrame frame={{1,2,3},0x24,0,0x80,false}; return frame;
}
static void fixture(Bno055 *imu,I2C_HandleTypeDef *i2c) {
    memset(imu,0,sizeof(*imu)); memset(metadata,0,sizeof(metadata));
    memset(payload,0,sizeof(payload));
    *i2c=(I2C_HandleTypeDef){HAL_I2C_STATE_READY,0};
    imu->i2c=i2c;imu->address=0x50;imu->present=true;imu->body_frame=identity();
    tick=100;reads=writes=0;latency=0;fail_reg=0xffff;legacy_read=false;
    metadata[0]=0x80;metadata[2]=8;metadata[6]=0x24;
    put16(payload,1600);put16(payload+2,-3200);put16(payload+4,800);
    put16(payload+6,1440);put16(payload+8,320);put16(payload+10,-160);
}

static void units(void) {
    Bno055 imu;I2C_HandleTypeDef i2c;fixture(&imu,&i2c);
    BodyImuState sample;float gyro[3],yaw;
    assert(bno_imu_decode(payload,metadata,&imu.body_frame,0,0,55,7,&sample,gyro,&yaw));
    assert(sample.valid && !sample.axis_verified && sample.timestamp_ms==55 && sample.sequence==7);
    closef(sample.gx,100.f*BNO_IMU_DEG_TO_RAD);closef(sample.gy,-200.f*BNO_IMU_DEG_TO_RAD);
    closef(gyro[2],50.f*BNO_IMU_DEG_TO_RAD);closef(yaw,90.f*BNO_IMU_DEG_TO_RAD);
    closef(sample.roll,-10.f*BNO_IMU_DEG_TO_RAD);closef(sample.pitch,20.f*BNO_IMU_DEG_TO_RAD);
    /* Gyro and Euler bits are independent, not an assumed default unit. */
    metadata[0]=0x82;put16(payload,900);put16(payload+2,-1800);
    assert(bno_imu_decode(payload,metadata,&imu.body_frame,10,-20,55,8,&sample,gyro,&yaw));
    closef(sample.gx,1.f);closef(sample.gy,-2.f);
    closef(sample.roll,-11.f*BNO_IMU_DEG_TO_RAD);closef(sample.pitch,22.f*BNO_IMU_DEG_TO_RAD);
    metadata[0]=0x84;put16(payload+8,450);put16(payload+10,-225);
    assert(bno_imu_decode(payload,metadata,&imu.body_frame,0,0,55,9,&sample,gyro,&yaw));
    closef(sample.roll,-.25f);closef(sample.pitch,.5f);
    closef(sample.gx,900.f*BNO_IMU_DEG_TO_RAD/16.f);
    metadata[0]=0x86;
    assert(bno_imu_decode(payload,metadata,&imu.body_frame,0,0,55,10,&sample,gyro,&yaw));
    closef(sample.gx,1.f);closef(sample.gy,-2.f);closef(sample.roll,-.25f);
    const uint8_t min_raw[]={0,128},max_raw[]={255,127};
    assert(bno_imu_i16le(min_raw)==-32768 && bno_imu_i16le(max_raw)==32767);
}

static void rotations(void) {
    Bno055 imu;I2C_HandleTypeDef i2c;fixture(&imu,&i2c);
    BodyImuState sample;float gyro[3];
    int count=0;
    for (int x=-3;x<=3;++x) for (int y=-3;y<=3;++y) for (int z=-3;z<=3;++z) {
        Bno055BodyFrame frame=identity();
        frame.gyro_axis[0]=(int8_t)x;frame.gyro_axis[1]=(int8_t)y;frame.gyro_axis[2]=(int8_t)z;
        if (!bno_imu_rotation_valid(frame.gyro_axis)) continue;
        ++count;frame.axis_verified=true;
        assert(bno_imu_decode(payload,metadata,&frame,0,0,1,1,&sample,gyro,NULL));
        assert(sample.axis_verified);
        for (int i=0;i<3;++i) {
            const int axis=frame.gyro_axis[i];const float raw[3]={1600,-3200,800};
            closef(gyro[i],raw[abs(axis)-1]*(axis<0 ? -1.f : 1.f)*BNO_IMU_DEG_TO_RAD/16.f);
        }
    }
    assert(count==24);
    const int8_t reflection[]={2,1,3},duplicate[]={1,1,3},bad[]={-128,2,3};
    assert(!bno_imu_rotation_valid(reflection));assert(!bno_imu_rotation_valid(duplicate));
    assert(!bno_imu_rotation_valid(bad));assert(!bno_imu_rotation_valid(NULL));
    const uint8_t configs[]={0x21,0x24,0x24,0x21,0x24,0x21,0x21,0x24};
    const uint8_t signs[]={4,0,6,2,3,1,7,5};
    for (unsigned i=0;i<8;++i) assert(bno_imu_device_rotation_valid(configs[i],signs[i]));
    assert(!bno_imu_device_rotation_valid(0x21,0));
    assert(!bno_imu_device_rotation_valid(0xff,0));
}

static void metadata_guard(void) {
    Bno055 imu;I2C_HandleTypeDef i2c;fixture(&imu,&i2c);
    BodyImuState sample;imu.body_frame.axis_verified=true;
    assert(bno_imu_decode(payload,metadata,&imu.body_frame,0,0,1,1,&sample,NULL,NULL));
    assert(sample.axis_verified);
    metadata[7]=6; /* valid hardware rotation, but not the verified bench state */
    assert(bno_imu_decode(payload,metadata,&imu.body_frame,0,0,1,1,&sample,NULL,NULL));
    assert(sample.valid && !sample.axis_verified);
    metadata[7]=0;metadata[0]=0;
    assert(bno_imu_decode(payload,metadata,&imu.body_frame,0,0,1,1,&sample,NULL,NULL));
    assert(!sample.axis_verified);
    metadata[2]=0; /* reset/config mode cannot masquerade as valid fused data */
    assert(!bno_imu_decode(payload,metadata,&imu.body_frame,0,0,1,1,&sample,NULL,NULL));
    assert(!sample.valid && !sample.axis_verified);
    metadata[2]=8;metadata[6]=0x3f;
    assert(!bno_imu_decode(payload,metadata,&imu.body_frame,0,0,1,1,&sample,NULL,NULL));
}

static void cache_and_heading(void) {
    Bno055 imu;I2C_HandleTypeDef i2c;fixture(&imu,&i2c);latency=1;
    BodyImuState first,second;
    assert(bno055_read_body_imu(&imu,&first));
    assert(reads==2 && writes==0 && first.timestamp_ms==102 && first.sequence==1);
    assert(first.valid && !first.axis_verified);
    int16_t yaw=0;assert(bno055_read_heading(&imu,&yaw));assert(abs(yaw-900)<=1);
    tick=111;assert(bno055_read_body_imu(&imu,&second));
    assert(reads==2 && memcmp(&first,&second,sizeof(first))==0);
    tick=112;assert(bno055_read_body_imu(&imu,&second));
    assert(reads==4 && second.timestamp_ms==114 && second.sequence==2);
    /* Even identical payloads are new *observations*, never a generation ID. */
    closef(first.gx,second.gx);
    imu.body_sequence=UINT32_MAX;imu.cached_body_imu.valid=false;tick=UINT32_MAX-1;
    assert(bno055_read_body_imu(&imu,&first));assert(first.timestamp_ms==0 && first.sequence==0);
    tick=9;assert(bno055_read_body_imu(&imu,&second));assert(second.timestamp_ms==0);
    tick=10;assert(bno055_read_body_imu(&imu,&second));assert(second.sequence==1);
}

static void failures(void) {
    Bno055 imu;I2C_HandleTypeDef i2c;fixture(&imu,&i2c);BodyImuState sample;
    assert(bno055_read_body_imu(&imu,&sample));tick+=20;const uint32_t sequence=sample.sequence;
    fail_reg=0x3b;assert(!bno055_read_body_imu(&imu,&sample));
    assert(!sample.valid && sample.sequence==sequence && !imu.cached_body_imu.valid);
    fail_reg=0x14;assert(!bno055_read_body_imu(&imu,&sample));
    assert(!sample.valid && sample.sequence==sequence);
    fail_reg=0xffff;metadata[2]=0;const uint32_t before=reads;
    assert(!bno055_read_body_imu(&imu,&sample));assert(reads==before+1);
    metadata[2]=8;imu.body_frame.gyro_axis[0]=0;
    assert(!bno055_read_body_imu(&imu,&sample));assert(sample.sequence==sequence);
    imu.body_frame=identity();assert(bno055_read_body_imu(&imu,&sample));
    assert(sample.sequence==sequence+1U && sample.valid);
    assert(!bno055_read_body_imu(NULL,&sample) && !sample.valid);
    assert(!bno055_read_body_imu(&imu,NULL));assert(writes==0);
}

static void busy_bound(void) {
    Bno055 imu;I2C_HandleTypeDef i2c;fixture(&imu,&i2c);BodyImuState sample;
    i2c.busy=1;assert(!bno055_read_body_imu(&imu,&sample));assert(reads==0 && tick==100);
    i2c.busy=0;i2c.state=HAL_I2C_STATE_BUSY_RX;
    assert(!bno055_read_body_imu(&imu,&sample));assert(reads==0 && tick==100);
    i2c.state=HAL_I2C_STATE_READY;fail_reg=0x3b;
    assert(!bno055_read_body_imu(&imu,&sample));assert(reads==1 && tick==104);
    fail_reg=0x14;assert(!bno055_read_body_imu(&imu,&sample));assert(reads==3 && tick==108);
}

static void config_and_legacy(void) {
    Bno055 imu;I2C_HandleTypeDef i2c;fixture(&imu,&i2c);BodyImuState sample;
    imu.level_valid=true;imu.level_roll_tenths=10;imu.level_pitch_tenths=-20;
    assert(bno055_read_body_imu(&imu,&sample));
    legacy_read=true;int16_t yaw,roll,pitch;
    assert(bno055_read_euler(&imu,&yaw,&roll,&pitch));
    closef(sample.roll,(float)roll*BNO_IMU_DEG_TO_RAD/10.f);
    closef(sample.pitch,(float)pitch*BNO_IMU_DEG_TO_RAD/10.f);
    assert(yaw==900 && roll==-110 && pitch==220);legacy_read=false;
    Bno055BodyFrame frame=identity();frame.gyro_axis[0]=2;
    assert(!bno055_set_body_frame(&imu,&frame));
    frame.gyro_axis[1]=-1;frame.axis_verified=true;
    const uint32_t before=reads;
    assert(bno055_set_body_frame(&imu,&frame));assert(!imu.cached_body_imu.valid && reads==before);
    assert(bno055_read_body_imu(&imu,&sample));assert(sample.axis_verified);
    closef(sample.gx,-200.f*BNO_IMU_DEG_TO_RAD);closef(sample.gy,-100.f*BNO_IMU_DEG_TO_RAD);
    assert(imu.level_roll_tenths==10 && imu.level_pitch_tenths==-20 && writes==0);
}

int main(int argc,char **argv) {
    assert(argc==2);
    if (!strcmp(argv[1],"units")) units();
    else if (!strcmp(argv[1],"rotations")) rotations();
    else if (!strcmp(argv[1],"metadata")) metadata_guard();
    else if (!strcmp(argv[1],"cache")) cache_and_heading();
    else if (!strcmp(argv[1],"failures")) failures();
    else if (!strcmp(argv[1],"busy")) busy_bound();
    else if (!strcmp(argv[1],"legacy")) config_and_legacy();
    else assert(0);
    puts("BNO055 sensor contract passed");return 0;
}

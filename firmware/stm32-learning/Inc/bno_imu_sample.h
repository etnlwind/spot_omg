#ifndef BNO_IMU_SAMPLE_H
#define BNO_IMU_SAMPLE_H

/* Sensor-only, HAL-free decoding. Bosch BNO055 datasheet rev 1.8, sections
 * 3.4, 3.6.1, 3.6.4 and 3.6.5; unit masks cross-checked against Bosch's
 * BNO055_driver/bno055.h (GYRO_UNIT_MSK=0x02, EULER_UNIT_MSK=0x04).
 */
#include "body_stabilizer.h"
#include <stddef.h>

#define BNO_IMU_DEG_TO_RAD (0.01745329251994329577f)

typedef struct {
    int8_t gyro_axis[3];
    uint8_t expected_axis_config, expected_axis_sign, expected_orientation;
    bool axis_verified;
} Bno055BodyFrame;

/* Checked as integers, including INT8_MIN: no negation overflow. */
static inline bool bno_imu_rotation_valid(const int8_t axes[3]) {
    if (!axes) return false;
    int row[3][3]={{0,0,0},{0,0,0},{0,0,0}};
    for (int i=0;i<3;++i) {
        const int axis=axes[i];
        if (axis==0 || axis < -3 || axis > 3) return false;
        row[i][(axis<0 ? -axis : axis)-1]=axis<0 ? -1 : 1;
    }
    const int determinant=
        row[0][0]*(row[1][1]*row[2][2]-row[1][2]*row[2][1])-
        row[0][1]*(row[1][0]*row[2][2]-row[1][2]*row[2][0])+
        row[0][2]*(row[1][0]*row[2][1]-row[1][1]*row[2][0]);
    return determinant==1;
}

static inline bool bno_imu_device_rotation_valid(uint8_t config,uint8_t sign) {
    if ((config&0xc0U)!=0 || (sign&0xf8U)!=0) return false;
    int8_t axes[3];
    for (unsigned i=0;i<3;++i) {
        const unsigned axis=(config>>(2U*i))&3U;
        if (axis>2U) return false;
        axes[i]=(int8_t)((sign&(1U<<(2U-i))) ? -(int)(axis+1U) : (int)(axis+1U));
    }
    return bno_imu_rotation_valid(axes);
}

static inline bool bno_imu_frame_valid(const Bno055BodyFrame *frame) {
    return frame && bno_imu_rotation_valid(frame->gyro_axis) &&
        bno_imu_device_rotation_valid(frame->expected_axis_config,
                                      frame->expected_axis_sign) &&
        (frame->expected_orientation&0x7fU)==0;
}

static inline int16_t bno_imu_i16le(const uint8_t *bytes) {
    const int32_t raw=(int32_t)bytes[0]+((int32_t)bytes[1]<<8);
    return (int16_t)(raw>=32768 ? raw-65536 : raw);
}

/* metadata is the eight-byte read from UNIT_SEL (0x3b) through AXIS_MAP_SIGN
 * (0x42). Requiring the expected fusion mode rejects reset/config-mode data.
 * It does not certify fusion calibration quality or sample generation age.
 */
static inline bool bno_imu_metadata_valid(const uint8_t metadata[8]) {
    return metadata && metadata[2]==0x08U && metadata[3]==0x00U &&
        bno_imu_device_rotation_valid(metadata[6],metadata[7]);
}

/* Twelve bytes from 0x14: actual gyro XYZ, then fused Euler heading/roll/pitch.
 * Body +X forward, +Y left, +Z up. The Euler mapping deliberately matches the
 * existing bench-mapped reader near level: body roll=sensor Euler pitch,
 * body pitch=sensor Euler roll. Only the explicit, verified gyro rotation can
 * certify their differential signs for the new PD controller.
 *
 * timestamp_ms is MCU receipt-completion time, NOT a BNO generation time.
 * sequence counts successful burst observations, NOT internal sensor samples.
 * The device offers neither a generation timestamp nor a sample counter.
 */
static inline bool bno_imu_decode(const uint8_t bytes[12],
                                 const uint8_t metadata[8],
                                 const Bno055BodyFrame *frame,
                                 int16_t level_roll_tenths,
                                 int16_t level_pitch_tenths,
                                 uint32_t timestamp_ms,uint32_t sequence,
                                 BodyImuState *out,float body_gyro[3],
                                 float *yaw_rad) {
    if (!out) return false;
    memset(out,0,sizeof(*out));
    out->timestamp_ms=timestamp_ms;
    out->sequence=sequence;
    if (!bytes || !bno_imu_metadata_valid(metadata) || !bno_imu_frame_valid(frame))
        return false;
    const float gyro_scale=(metadata[0]&0x02U) ? (1.f/900.f) : (BNO_IMU_DEG_TO_RAD/16.f);
    const float euler_scale=(metadata[0]&0x04U) ? (1.f/900.f) : (BNO_IMU_DEG_TO_RAD/16.f);
    float gyro[3];
    for (int i=0;i<3;++i) {
        const int axis=frame->gyro_axis[i];
        gyro[i]=(float)bno_imu_i16le(bytes+2*((axis<0 ? -axis : axis)-1))*
            gyro_scale*(axis<0 ? -1.f : 1.f);
        if (body_gyro) body_gyro[i]=gyro[i];
    }
    out->gx=gyro[0];
    out->gy=gyro[1];
    out->roll=(float)bno_imu_i16le(bytes+10)*euler_scale-
        (float)level_roll_tenths*(BNO_IMU_DEG_TO_RAD/10.f);
    out->pitch=(float)bno_imu_i16le(bytes+8)*euler_scale-
        (float)level_pitch_tenths*(BNO_IMU_DEG_TO_RAD/10.f);
    if (yaw_rad) *yaw_rad=(float)bno_imu_i16le(bytes+6)*euler_scale;
    out->valid=true;
    out->axis_verified=frame->axis_verified &&
        metadata[6]==frame->expected_axis_config &&
        metadata[7]==frame->expected_axis_sign &&
        (metadata[0]&0x80U)==frame->expected_orientation;
    return true;
}

#endif

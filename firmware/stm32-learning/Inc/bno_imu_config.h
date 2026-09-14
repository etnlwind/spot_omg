#ifndef BNO_IMU_CONFIG_H
#define BNO_IMU_CONFIG_H

/* These are SOFTWARE mappings of already-remapped BNO055 output channels.
 * Signed 1/2/3 means + or - sensor X/Y/Z. The three rows MUST form a proper
 * orthonormal rotation (determinant +1). Do not copy the legacy Euler name
 * swap here: Euler names are not an angular-velocity rotation matrix.
 *
 * Identity is an UNVERIFIED placeholder, not a measured mounting transform.
 * Keep VERIFIED=0 until the three-axis, torque-off bench procedure in
 * docs/BNO055-BODY-IMU.md passes. No BNO register is changed by these settings.
 */
#define BNO055_BODY_GYRO_X_AXIS (+1)
#define BNO055_BODY_GYRO_Y_AXIS (+2)
#define BNO055_BODY_GYRO_Z_AXIS (+3)
#define BNO055_BODY_AXES_VERIFIED 0

/* Register readback associated with the eventual verified bench mounting.
 * UNIT_SEL bit 7 controls the Euler convention. Gyro/Euler unit bits may
 * differ: the reader converts either documented unit choice to SI.
 */
#define BNO055_BODY_EXPECT_AXIS_CONFIG 0x24U
#define BNO055_BODY_EXPECT_AXIS_SIGN   0x00U
#define BNO055_BODY_EXPECT_ORIENTATION 0x80U

/* One sensor transaction uses <=3 tick milliseconds after the BUSY guard.
 * HAL's strict 'elapsed > timeout' test adds up to one tick. Two transactions
 * per uncached observation: metadata, then gyro+Euler. No retries or delays.
 */
#define BNO055_BODY_READ_TIMEOUT_MS 3U
#define BNO055_BODY_CACHE_MS        10U

#endif

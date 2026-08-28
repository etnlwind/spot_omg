#ifndef MOTOR_CAPABILITY_H
#define MOTOR_CAPABILITY_H

/*
 * Mixed actuator capability/configuration for the 12 V robot:
 * J1/J3 use STS3215 30kg; J2 uses STS3250 50kg.
 *
 * This file deliberately sits outside gait_policy.h.  The shared policy says
 * where a canonical joint should be; this file says how quickly the physical
 * actuator may be asked to get there.  Host analysis reads these same values,
 * so the regression limit and the firmware limiter cannot silently diverge.
 */

/* Approximate no-load data-sheet speed at 12 V. */
#define MOTOR_STS3215_MIN_POSITION 0U
#define MOTOR_STS3215_MAX_POSITION 4095U
#define MOTOR_STS3215_STEPS_PER_REVOLUTION 4096U
#define MOTOR_STS3215_NOMINAL_MAX_VELOCITY_DEG_S 270.0f

/*
 * A 5% analysis-only allowance covers 20 ms finite-difference quantisation
 * and normal unit-to-unit/no-load specification tolerance.  It is not a
 * command target: anything above the nominal limit is still reported.
 */
#define MOTOR_STS3215_REGRESSION_TRANSIENT_MARGIN 1.05f

/*
 * Trot3 commands at 90% of nominal speed.  The remaining 10% is headroom for
 * body load, bus latency and the 12 V rail sagging below its nominal value.
 */
#define MOTOR_STS3215_COMMAND_VELOCITY_LIMIT_DEG_S 243.0f

/*
 * Reach the command velocity limit in 60 ms (three 50 Hz frames).  This is a
 * software trajectory bound, not the STS3215 acceleration-register unit.
 */
#define MOTOR_STS3215_COMMAND_ACCELERATION_LIMIT_DEG_S2 4050.0f

/* STS3250: 0.133s/60deg at 12V = about 451deg/s, with 10% headroom. */
#define MOTOR_STS3250_NOMINAL_MAX_VELOCITY_DEG_S 451.0f
#define MOTOR_STS3250_COMMAND_VELOCITY_LIMIT_DEG_S 406.0f
#define MOTOR_STS3250_COMMAND_ACCELERATION_LIMIT_DEG_S2 6767.0f

/*
 * The outer trot3 limiter only works when the servo's inner position profile
 * is not the tighter bottleneck.  Use the controller's full profile and let
 * the canonical software limiter provide the smooth 243 deg/s command.
 */
#define MOTOR_STS3215_TROT3_REQUIRED_PROFILE_SPEED 3400U
#define MOTOR_STS3215_TROT3_REQUIRED_PROFILE_ACCELERATION 254U

/* About 8.4 degrees at 4096 ticks/revolution: lag, but not yet a stall. */
#define MOTOR_STS3215_TRACKING_LAG_THRESHOLD_TICKS 96U
#define MOTOR_STS3215_TRACKING_LAG_SAMPLES_FOR_DERATE 2U

/* A sampled rail at or below 11 V is considered a meaningful 12 V droop. */
#define MOTOR_STS3215_VOLTAGE_DROOP_THRESHOLD_MV 11000U

#endif

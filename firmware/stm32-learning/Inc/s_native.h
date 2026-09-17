#ifndef S_NATIVE_H
#define S_NATIVE_H
#include "gait_policy.h"
typedef struct {
    float previous[4][3], normal[4][3], entry_phase, last_phase;
    float stop_progress, stop_j1[4], support[64];
    float stop_pose[4][3], stop_feet[4][3];
    bool stop_first_fl_rr;
    float support_linear, support_yaw;
    float extended_rear_m;
    unsigned diagnostic_leg; /* 0 disabled, 3 RL, 4 RR, 5 all legs */
    float diagnostic_lift_extra_m;
    bool diagnostic_width_active;
    bool diagnostic_fr_extra;
    float diagnostic_width_m;
    bool have_phase, have_support, continuous_recovery, extended_reach, placement_swing, uniform_recovery, early_fold_recovery;
} SNativeControl;
void s_native_reset(SNativeControl *s);
/* false preserves V6.1; true selects the independent V6.2.1 trajectory. */
void s_native_reset_profile(SNativeControl *s, bool continuous_recovery);
/* V6.2.3 preserves older trajectories and adds smooth lift / rear reach. */
void s_native_reset_v623(SNativeControl *s);
void s_native_reset_v624(SNativeControl *s);
void s_native_reset_v625(SNativeControl *s);
void s_native_reset_v626(SNativeControl *s);
void s_native_reset_v627(SNativeControl *s);
void s_native_stand(GaitPolicyLegTarget out[4]);
/* yaw/requested_yaw follow drive protocol: positive turns the body right. */
bool s_native_step(SNativeControl *s, float phase, float amplitude, float linear, float yaw,
                   float requested_linear, float requested_yaw, float dt, bool stopping,
                   GaitPolicyLegTarget out[4]);
bool s_native_stopped(const SNativeControl *s);
/* Exported for host geometry regression; no simulator state enters control. */
void s_native_foot(int leg, const float q[3], float point[3], float jac[3][3]);
#endif

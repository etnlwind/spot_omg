#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "body_stabilization_config.h"

static const float feet[4][3]={{.2f,.1f,-.2f},{.2f,-.1f,-.2f},
                              {-.2f,.1f,-.2f},{-.2f,-.1f,-.2f}};
static const float all_stance[4]={1,1,1,1};
static int groups=0;
static bool close_to(float a,float b) {return fabsf(a-b)<1.e-6f;}
static BodyStabilizerConfig fast_config(void) {
    BodyStabilizerConfig c=body_stabilizer_default_config();
    c.angle_alpha=c.gyro_alpha=0;c.slew_rad_s=2;
    return c;
}
static BodyImuState reading(float roll,float pitch,float gx,float gy,uint32_t now,uint32_t seq) {
    BodyImuState v={roll,pitch,gx,gy,now,seq,true,true};return v;
}
static void bounded(const float out[4][3]) {
    for(int i=0;i<4;i++)for(int j=0;j<3;j++) {
        assert(isfinite(out[i][j]));assert(fabsf(out[i][j])<=.005001f);
        if(j<2)assert(out[i][j]==0.f);
    }
}
static void test_p_sign_and_nonzero_target(void) {
    BodyStabilizerState s={0};BodyStabilizerConfig c=fast_config();float out[4][3];
    c.kp_roll=c.kp_pitch=.5f;c.kd_roll_s=c.kd_pitch_s=0;
    c.target_roll_rad=.02f;
    BodyImuState v=reading(.04f,0,0,0,0,0);
    assert(body_stabilizer_update(&s,&c,&v,0,.02f,true,feet,all_stance,out));
    assert(close_to(s.diagnostics.error[0],-.02f));assert(close_to(s.u[0],-.01f));
    assert(close_to(out[0][2],.001f)&&close_to(out[1][2],-.001f));
    assert(close_to(out[2][2],.001f)&&close_to(out[3][2],-.001f));
    body_stabilizer_reset(&s);c.target_roll_rad=0;v=reading(0,.04f,0,0,0,0);
    assert(body_stabilizer_update(&s,&c,&v,0,.02f,true,feet,all_stance,out));
    assert(close_to(out[0][2],-.004f)&&close_to(out[1][2],-.004f));
    assert(close_to(out[2][2],.004f)&&close_to(out[3][2],.004f));
    bounded(out);groups++;
}
static void test_real_gyro_damping_without_angle_change(void) {
    BodyStabilizerState s={0};BodyStabilizerConfig c=fast_config();float out[4][3];
    c.kp_roll=c.kp_pitch=0;c.kd_roll_s=c.kd_pitch_s=.1f;
    BodyImuState v=reading(0,0,.2f,0,0,0);
    assert(body_stabilizer_update(&s,&c,&v,0,.02f,true,feet,all_stance,out));
    assert(close_to(s.u[0],-.02f));assert(out[0][2]>0&&out[1][2]<0);
    body_stabilizer_reset(&s);v=reading(0,0,0,.2f,0,0);
    assert(body_stabilizer_update(&s,&c,&v,0,.02f,true,feet,all_stance,out));
    assert(out[0][2]<0&&out[2][2]>0);
    assert(s.diagnostics.raw[0]==0&&s.diagnostics.raw[2]==0&&s.diagnostics.raw[3]==.2f);
    groups++;
}
static void test_separate_axis_clamps_and_foot_bound(void) {
    BodyStabilizerState s={0};BodyStabilizerConfig c=fast_config();float out[4][3];
    c.kp_roll=c.kp_pitch=2;c.max_roll_rad=.03f;c.max_pitch_rad=.04f;c.max_error_rad=.4f;
    BodyImuState v=reading(.3f,.3f,0,0,0,0);
    assert(body_stabilizer_update(&s,&c,&v,0,.02f,true,feet,all_stance,out));
    assert(close_to(s.u[0],-.03f)&&close_to(s.u[1],-.04f));
    assert(s.diagnostics.axis_clamp_mask==3);
    assert(s.diagnostics.foot_clamp_mask!=0);bounded(out);
    assert(close_to(out[1][2],-.005f));groups++;
}
static void test_lpf_alpha_means_previous_weight_and_duplicate_is_not_new(void) {
    BodyStabilizerState s={0};BodyStabilizerConfig c=fast_config();float out[4][3];
    c.angle_alpha=.5f;c.gyro_alpha=.2f;
    BodyImuState v=reading(0,0,0,0,0,1);
    assert(body_stabilizer_update(&s,&c,&v,0,.02f,true,feet,all_stance,out));
    v=reading(.04f,0,.4f,0,10,2);
    assert(body_stabilizer_update(&s,&c,&v,10,.01f,true,feet,all_stance,out));
    assert(close_to(s.filtered[0],.02f)&&close_to(s.filtered[2],.32f));
    assert(s.diagnostics.new_sample);
    /* Deliberately altered cached payload must not masquerade as new data. */
    v.roll=.08f;v.timestamp_ms=20;
    assert(body_stabilizer_update(&s,&c,&v,20,.01f,true,feet,all_stance,out));
    assert(!s.diagnostics.new_sample&&s.sample_timestamp_ms==10);
    assert(close_to(s.filtered[0],.02f)&&close_to(s.diagnostics.raw[0],.08f));
    v.timestamp_ms=111;
    assert(body_stabilizer_update(&s,&c,&v,111,.02f,true,feet,all_stance,out));
    assert(s.diagnostics.status==BODY_STABILIZER_STALE&&s.sample_timestamp_ms==10);
    groups++;
}
static void test_timestamp_and_sequence_wrap_and_replay(void) {
    BodyStabilizerState s={0};BodyStabilizerConfig c=fast_config();float out[4][3];
    BodyImuState v=reading(.02f,0,0,0,UINT32_MAX-20U,UINT32_MAX);
    assert(body_stabilizer_update(&s,&c,&v,UINT32_MAX-20U,.02f,true,feet,all_stance,out));
    v.timestamp_ms=5;v.sequence=0;
    assert(body_stabilizer_update(&s,&c,&v,5,.02f,true,feet,all_stance,out));
    assert(s.diagnostics.status==BODY_STABILIZER_ACTIVE&&s.diagnostics.new_sample);
    assert(s.diagnostics.sample_age_ms==0);
    v.timestamp_ms=UINT32_MAX-10U;v.sequence=1;
    assert(body_stabilizer_update(&s,&c,&v,10,.02f,true,feet,all_stance,out));
    assert(s.diagnostics.status==BODY_STABILIZER_REPLAYED_SAMPLE&&s.sample_timestamp_ms==5);
    v.timestamp_ms=30;v.sequence=2;
    assert(body_stabilizer_update(&s,&c,&v,20,.02f,true,feet,all_stance,out));
    assert(s.diagnostics.status==BODY_STABILIZER_INVALID_SAMPLE);bounded(out);groups++;
}
static void test_off_and_sensor_loss_fade_without_swing_leak(void) {
    BodyStabilizerState s={0};BodyStabilizerConfig c=fast_config();float out[4][3];
    c.slew_rad_s=.1f;c.kp_roll=.5f;
    for(unsigned i=0;i<10;i++) {
        BodyImuState v=reading(.04f,0,0,0,i*20,i);
        assert(body_stabilizer_update(&s,&c,&v,i*20,.02f,true,feet,all_stance,out));
    }
    assert(close_to(s.u[0],-.02f));
    const float two_stance[4]={1,0,0,1};
    for(unsigned i=0;i<12;i++) {
        float old=s.u[0];BodyImuState v=reading(.04f,0,0,0,200+i*20,10+i);
        assert(body_stabilizer_update(&s,&c,i<5?&v:NULL,200+i*20,.02f,i>=5,
                                      feet,two_stance,out));
        assert(fabsf(s.u[0]-old)<=.002001f);
        assert(fabsf(s.u[0])<=fabsf(old)+1.e-7f);
        assert(out[1][2]==0.f&&out[2][2]==0.f);bounded(out);
    }
    assert(close_to(s.u[0],0));groups++;
}
static void test_invalid_sensor_guards_keep_finite_output_and_filter(void) {
    BodyStabilizerConfig c=fast_config();c.slew_rad_s=.1f;
    for(int which=0;which<5;which++) {
        BodyStabilizerState s={0};float out[4][3];
        BodyImuState v=reading(.03f,0,0,0,0,1);
        assert(body_stabilizer_update(&s,&c,&v,0,.02f,true,feet,all_stance,out));
        float old=s.u[0],filtered=s.filtered[0];v.timestamp_ms=20;v.sequence=2;
        BodyStabilizerStatus expected;
        if(which==0){v.roll=NAN;expected=BODY_STABILIZER_INVALID_SAMPLE;}
        else if(which==1){v.valid=false;expected=BODY_STABILIZER_INVALID_SAMPLE;}
        else if(which==2){v.axis_verified=false;expected=BODY_STABILIZER_AXES_UNVERIFIED;}
        else if(which==3){v.roll=.5f;expected=BODY_STABILIZER_ANGLE_GUARD;}
        else {v.gx=5.f;expected=BODY_STABILIZER_GYRO_GUARD;}
        assert(body_stabilizer_update(&s,&c,&v,20,.02f,true,feet,all_stance,out));
        assert(s.diagnostics.status==expected);
        assert(fabsf(s.u[0])<=fabsf(old));assert(s.filtered[0]==filtered);bounded(out);
        if(which==0)assert(!s.diagnostics.input_finite&&isnan(s.diagnostics.raw[0]));
    }
    groups++;
}
static void test_bad_math_contract_does_not_advance_state(void) {
    BodyStabilizerState s={0};BodyStabilizerConfig c=fast_config();float out[4][3];
    BodyImuState v=reading(.04f,0,0,0,0,1);
    assert(body_stabilizer_update(&s,&c,&v,0,.02f,true,feet,all_stance,out));
    float old=s.u[0];v.timestamp_ms=20;v.sequence=2;
    assert(!body_stabilizer_update(&s,&c,&v,20,NAN,true,feet,all_stance,out));
    assert(s.diagnostics.status==BODY_STABILIZER_INVALID_DT&&s.u[0]==old);bounded(out);
    assert(!body_stabilizer_update(&s,&c,&v,20,0,true,feet,all_stance,out));bounded(out);
    float bad_feet[4][3];memcpy(bad_feet,feet,sizeof(feet));bad_feet[0][1]=NAN;
    assert(!body_stabilizer_update(&s,&c,&v,20,.02f,true,bad_feet,all_stance,out));
    assert(s.u[0]==old&&s.sample_sequence==1);bounded(out);
    float bad_weights[4]={1,NAN,0,1};
    assert(!body_stabilizer_update(&s,&c,&v,20,.02f,true,feet,bad_weights,out));bounded(out);
    c.angle_alpha=1;
    assert(!body_stabilizer_update(&s,&c,&v,20,.02f,true,feet,all_stance,out));
    assert(s.diagnostics.status==BODY_STABILIZER_INVALID_CONFIG);bounded(out);groups++;
}
static void test_phase_weight_continuity_and_exact_swing_zero(void) {
    float w[4];const float offsets[4]={0,.5f,.5f,0};
    for(int j=0;j<500;j++) {
        float phase=j/500.f;
        assert(body_stabilizer_stance_weights(phase,.6f,1.2f,.08f,w));
        assert(w[0]==w[3]&&w[1]==w[2]);
        for(int i=0;i<4;i++) {
            float q=phase+offsets[i];q-=floorf(q);
            assert(w[i]>=0&&w[i]<=1);
            if(q>=.6f)assert(w[i]==0.f);
        }
    }
    for(int i=0;i<2;i++) {
        float boundary=i==0?.6f:1.f;
        assert(body_stabilizer_stance_weights(boundary-1.e-5f,.6f,1.2f,.08f,w));
        assert(w[0]<1.e-8f);
        assert(body_stabilizer_stance_weights(boundary+1.e-5f,.6f,1.2f,.08f,w));
        assert(w[0]<1.e-8f);
    }
    assert(!body_stabilizer_stance_weights(NAN,.6f,1.2f,.08f,w));
    for(int i=0;i<4;i++)assert(w[i]==0.f);
    groups++;
}
int main(void) {
    BodyStabilizerState reset_state;
    memset(&reset_state,0x5a,sizeof(reset_state));
    body_stabilizer_reset(&reset_state);
    assert(reset_state.diagnostics.status==BODY_STABILIZER_NO_SAMPLE);
    assert(!reset_state.initialized && reset_state.u[0]==0 && reset_state.u[1]==0);
    assert(body_stabilizer_config_valid(&BODY_STABILIZATION_DEFAULT_CONFIG));
    test_p_sign_and_nonzero_target();test_real_gyro_damping_without_angle_change();
    test_separate_axis_clamps_and_foot_bound();
    test_lpf_alpha_means_previous_weight_and_duplicate_is_not_new();
    test_timestamp_and_sequence_wrap_and_replay();
    test_off_and_sensor_loss_fade_without_swing_leak();
    test_invalid_sensor_guards_keep_finite_output_and_filter();
    test_bad_math_contract_does_not_advance_state();
    test_phase_weight_continuity_and_exact_swing_zero();
    printf("body_stabilizer: %d standalone groups passed\n",groups);
    return 0;
}

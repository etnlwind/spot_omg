#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H
#include <stdbool.h>
#include <math.h>

typedef struct {
    float reference, error, integral, correction, settling;
    bool active;
} HeadingControl;

static inline float heading_clip(float x, float limit) {
    return fmaxf(-limit, fminf(limit, x));
}
static inline float heading_wrap(float x) {
    return x - 360.f * floorf((x + 180.f) / 360.f);
}
/* Heading is CCW-positive in robot coordinates; drive yaw is right-positive.
 * Call every 20 ms with a fresh-or-age-checked IMU observation. */
static inline float heading_update(HeadingControl *s, float heading,
        bool valid, bool enabled, float linear, float manual_yaw,
        float applied_yaw, float dt) {
    if (!valid || !enabled || !isfinite(heading) || !isfinite(linear) || !isfinite(manual_yaw) ||
            !isfinite(applied_yaw) || !isfinite(dt) || linear <= .15f ||
            fabsf(manual_yaw) > .0001f || dt <= 0.f || dt > .1f) {
        *s = (HeadingControl){0};
        return 0.f;
    }
    if (!s->active) {
        if (fabsf(applied_yaw) > .02f) s->settling = 0.f;
        else s->settling += dt;
        if (s->settling < .2f) return 0.f;
        s->reference = heading;
        s->active = true;
    }
    float raw = heading_wrap(heading - s->reference);
    s->error += .2f * (raw - s->error);
    float error = fabsf(s->error) < .3f ? 0.f : s->error;
    float candidate = heading_clip(s->integral + .006f * error * dt, .15f);
    float wanted = .035f * error + candidate;
    if (fabsf(wanted) < .25f || wanted * error < 0.f) s->integral = candidate;
    wanted = heading_clip(.035f * error + s->integral, .25f);
    s->correction += heading_clip(wanted - s->correction, .5f * dt);
    return s->correction;
}
#endif


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#define MAX_PWM_LIMIT               950
#define PWM_KINETIC_L        200.0f
#define PWM_KINETIC_R        250.0f
#define PWM_PER_TICK_S       0.8f
#define STICK_MIN_S          0.15f
#define STICK_TICKS          2.5f
#define STICTION_RATE        900.0f
#define STICTION_MAX         300.0f
#define STICTION_FADE        1500.0f
#define SPEED_KP             1.0f
#define SPEED_KI_UP          20.0f
#define SPEED_KI_DOWN        8.0f
#define INTEGRAL_LIMIT_PWM   120.0f
#define INTEGRAL_BACKOFF_PWM 150.0f
typedef struct {
    float kinetic;        // PWM that keeps this wheel rolling (PWM_KINETIC_*)
    float integral;
    float boost;          // break-away push, PWM
    float since_tick_s;   // time since the last encoder tick in the commanded direction
    int32_t last_count;
    int16_t last_target;
} wheel_ctrl_t;

static wheel_ctrl_t ctrl_l = {.kinetic = PWM_KINETIC_L}, ctrl_r = {.kinetic = PWM_KINETIC_R};

// Returns PWM in the wheel's forward-positive sense; the caller applies
// MOTOR_DIR. target and measured are both ticks/s, forward-positive; count
// is the wheel's encoder total (forward-positive).
static float wheel_ctrl_update(wheel_ctrl_t *c, int16_t target, float measured,
                               int32_t count, float dt) {
    int32_t moved = count - c->last_count;
    c->last_count = count;
    bool reversed = (c->last_target != 0) && ((target > 0) != (c->last_target > 0));
    if (target == 0 || reversed) {
        c->integral = 0.0f;
        c->boost = 0.0f;
        c->since_tick_s = 0.0f;
    }
    c->last_target = target;
    if (target == 0) { return 0.0f; }

    float sign = (target > 0) ? 1.0f : -1.0f;
    float err = (float)target - measured;

    // Static friction: no tick in the commanded direction for a few expected
    // tick intervals means the wheel is stuck, not just slow.
    if ((float)moved * sign > 0.0f) {
        c->since_tick_s = 0.0f;
    } else {
        c->since_tick_s += dt;
    }
    float stuck_after = fmaxf(STICK_MIN_S, STICK_TICKS / fabsf((float)target));
    bool stuck = c->since_tick_s > stuck_after;
    if (stuck) {
        c->boost = fminf(STICTION_MAX, c->boost + STICTION_RATE * dt);
    } else {
        c->boost = fmaxf(0.0f, c->boost - STICTION_FADE * dt);
    }

    float out = sign * (c->kinetic + PWM_PER_TICK_S * fabsf((float)target) + c->boost)
              + SPEED_KP * err + c->integral;

    bool saturated = (fabsf(out) >= (float)MAX_PWM_LIMIT) && ((out > 0.0f) == (err > 0.0f));
    // Not while stuck: breaking away is the push's job. An integral wound up
    // during a 0.5 s stick (it reached its limit) added to the push at
    // break-away and ran the robot at 0.5 m/s for a 0.15 m/s target
    // (2026-10-16 ground test).
    if (!saturated && !stuck) {
        c->integral += ((err * sign > 0.0f) ? SPEED_KI_UP : SPEED_KI_DOWN) * err * dt;
        // Measured along the commanded direction. Friction is handled by the
        // feedforward and the break-away push, so the integral only trims;
        // it is limited, never dumped (dumping it on the first overshoot made
        // the wheels stick-slip under the full weight on two motors).
        float i_dir = c->integral * sign;
        if (i_dir > INTEGRAL_LIMIT_PWM) { i_dir = INTEGRAL_LIMIT_PWM; }
        else if (i_dir < -INTEGRAL_BACKOFF_PWM) { i_dir = -INTEGRAL_BACKOFF_PWM; }
        c->integral = i_dir * sign;
    }

    if (out > MAX_PWM_LIMIT) { out = MAX_PWM_LIMIT; }
    else if (out < -MAX_PWM_LIMIT) { out = -MAX_PWM_LIMIT; }
    // Overshoot is corrected by coasting, not by driving the motor backward.
    if (out * sign < 0.0f) { out = 0.0f; }
    return out;
}

/* Plant: measured friction (rolling K0 + slope*v, break-away STATIC), time constant TAU. */
static double K0 = 215, STATIC = 365, SLOPE = 0.8, TAU = 0.15, K0_FAST = 215;

int main(int argc, char **argv) {
    double target_mps = atof(argv[1]); int dir = atoi(argv[2]);
    if (argc > 3) K0 = atof(argv[3]);
    if (argc > 4) STATIC = atof(argv[4]);
    K0_FAST = (argc > 6) ? atof(argv[6]) : K0;
    float kinetic = (argc > 5) ? (float)atof(argv[5]) : PWM_KINETIC_L;
    double m_per_tick = 3.14159265 * 0.13 / 74.0;
    int16_t target = (int16_t)lround(dir * target_mps / m_per_tick);
    wheel_ctrl_t c = {.kinetic = kinetic};
    double v = 0, pos = 0; bool stuck = true; double pwm = 0;
    double hist_t[6] = {0}; long hist_c[6] = {0}; int n = 0;
    for (int ms = 0; ms < 4000; ms++) {
        double t = ms * 1e-3;
        if (ms % 20 == 0) {
            long count = (long)floor(pos);
            for (int i = 5; i > 0; i--) { hist_t[i] = hist_t[i-1]; hist_c[i] = hist_c[i-1]; }
            hist_t[0] = t; hist_c[0] = count; if (n < 6) n++;
            float measured = (n > 1) ? (float)((hist_c[0] - hist_c[n-1]) / (hist_t[0] - hist_t[n-1])) : 0.0f;
            pwm = wheel_ctrl_update(&c, target, measured, (int32_t)count, 0.02f);
            if (ms % 20 == 0)
                printf("%4.1fs pwm %7.1f  v %+6.3f m/s  boost %5.1f  I %7.1f\n", t, pwm, v * m_per_tick, c.boost, c.integral);
        }
        /* plant in the wheel's own sense: drive torque = pwm * dir */
        double drive = pwm;                       /* pwm is already forward-positive */
        double speed = v * dir;                   /* speed along commanded dir */
        if (stuck) { if (fabs(drive) > STATIC) stuck = false; else continue; }
        double k0 = K0 - (K0 - K0_FAST) * fmin(1.0, fabs(v) * m_per_tick / 0.2);
        double acc = (fabs(drive) - k0 - SLOPE * fabs(v)) / (SLOPE * TAU);
        if (drive * v < 0) acc = -(K0 + fabs(drive)) / (SLOPE * TAU);  /* pushing against motion */
        double newspeed = fabs(v) + acc * 1e-3;
        if (newspeed <= 0) { v = 0; stuck = true; } else { v = (v != 0 ? (v > 0 ? 1 : -1) : (drive > 0 ? 1 : -1)) * newspeed; }
        pos += v * 1e-3; (void)speed;
    }
    return 0;
}

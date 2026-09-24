/* Build and run on a PC (not on the board), from the repository root:
 *   gcc -O2 -IAssignment/CG2028_Assignment/Core/Inc Part2_Simulation/fall_sim.c \
 *       Assignment/CG2028_Assignment/Core/Src/fall_detector.c -lm -o fall_sim && ./fall_sim
 */
/* Host-side simulation: synthetic 50 Hz IMU traces -> C EWMA -> fall detector.
 * Orientation is modelled as a rotation theta about the sensor X axis:
 * gravity in sensor frame = (0, 1000cos(theta), 1000sin(theta)) mg, gyro X = dtheta/dt. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "fall_detector.h"

#define ALPHA_A 50
#define ALPHA_G 30
#define DT 0.02

static int ewma(int n, int o, int a) { return (a * n + (100 - a) * o) / 100; }
static double gauss(void) { double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = rand() / (RAND_MAX + 1.0); return sqrt(-2 * log(u)) * cos(2 * M_PI * v); }


static fall_detector_t fd;
static int fa[3], fg[3];
static unsigned t_ms;
static int events[16];

static const char *ev_name[] = {"NONE","CALIBRATED","FREEFALL","IMPACT","FALL_CONFIRMED","NEAR_FALL_REJECTED","RECOVERED","ACK","LONG_LIE","SOS"};

static void feed(double theta, double omega_dps, double grav_scale, double lx, double ly, double lz, double gx_extra, double gy_extra, int verbose)
{
    int raw_a[3], raw_g[3];
    raw_a[0] = (int)(lx + 15 * gauss());
    raw_a[1] = (int)(1000 * grav_scale * cos(theta) + ly + 15 * gauss());
    raw_a[2] = (int)(1000 * grav_scale * sin(theta) + lz + 15 * gauss());
    raw_g[0] = (int)((omega_dps + 1.5 * gauss()) * 1000);
    raw_g[1] = (int)((gx_extra + 1.5 * gauss()) * 1000);
    raw_g[2] = (int)((gy_extra + 1.5 * gauss()) * 1000);
    for (int i = 0; i < 3; i++) {
        if (raw_a[i] > 7999) raw_a[i] = 7999;
        if (raw_a[i] < -7999) raw_a[i] = -7999;
        fa[i] = ewma(raw_a[i], fa[i], ALPHA_A);
        fg[i] = ewma(raw_g[i], fg[i], ALPHA_G);
    }
    fd_event_t e = fd_update(&fd, fa, fg, t_ms);
    if (e != FD_EVENT_NONE) {
        events[e]++;
        if (verbose) {
            printf("    %6.2fs %-18s |a|=%4d |w|=%4d tilt=%3d", t_ms / 1000.0, ev_name[e], fd.feat.acc_mag_mg, fd.feat.gyro_mag_dps, fd.feat.tilt_deg);
            if (e == FD_EVENT_FALL_CONFIRMED || e == FD_EVENT_NEAR_FALL_REJECTED)
                printf("  rot=%d posture=%d still=%d%% %s", fd.last_rotation_dps, fd.last_posture_deg, fd.last_still_pct, fd.last_reason);
            printf("\n");
        }
    }
    t_ms += 20;
}

static double th; /* current orientation (rad) */

static void hold(double seconds, int v) { for (double s = 0; s < seconds; s += DT) feed(th, 0, 1, 0, 0, 0, 0, 0, v); }

/* rotate by delta_deg over seconds with smooth (sin^2) velocity profile; grav_scale lets |a| dip */
static void rotate(double delta_deg, double seconds, double grav_scale, int v)
{
    int n = (int)(seconds / DT);
    double d = delta_deg * M_PI / 180;
    for (int i = 0; i < n; i++) {
        double w = d / seconds * 2 * pow(sin(M_PI * (i + 0.5) / n), 2); /* rad/s, integrates to d */
        th += w * DT;
        feed(th, w * 180 / M_PI, grav_scale, 0, 0, 0, 0, 0, v);
    }
}

/* impact deceleration acts along the vertical (gravity direction) */
static void spike(double peak_mg, int samples, int v) { for (int i = 0; i < samples; i++) feed(th, 0, peak_mg / 1000.0, 0, 0, 0, 0, 0, v); }

static void reset(void) { memset(events, 0, sizeof events); memset(fa, 0, sizeof fa); memset(fg, 0, sizeof fg); t_ms = 0; th = 0; fd_init(&fd, 0); hold(2.0, 0); }

static int run(const char *name, void (*scenario)(int), int expect_fall, int reps)
{
    int ok = 0;
    for (int r = 0; r < reps; r++) {
        srand(1000 + r * 7);
        reset();
        scenario(r == 0);
        int fall = events[FD_EVENT_FALL_CONFIRMED] > 0;
        if (fall == expect_fall) ok++;
    }
    printf("%-44s expect %-8s -> %d/%d correct\n\n", name, expect_fall ? "FALL" : "no fall", ok, reps);
    return ok == reps;
}

/* ---------------- Scenarios ---------------- */
static void s_still(int v) { hold(10, v); }
static void s_fall_freefall(int v) {         /* trip forward: partial free fall while rotating 90deg, hit floor, lie still */
    rotate(20, 0.25, 0.9, v); rotate(70, 0.35, 0.25, v); spike(3500, 2, v); spike(1800, 1, v); hold(4, v); }
static void s_fall_no_freefall(int v) {      /* slump/slide sideways: |a| stays ~1g, rotation 90deg in 0.6s, 2.8g impact */
    rotate(85, 0.6, 0.85, v); spike(2800, 2, v); hold(4, v); }
static void s_fall_backward_soft(int v) {    /* backwards onto cushion: softer 2.4 g impact */
    rotate(-80, 0.5, 0.4, v); spike(2400, 2, v); hold(4, v); }
static void s_sit_down(int v) {              /* torso leans ~25deg forward then back, lands in chair 1.4 g */
    rotate(25, 0.8, 0.9, v); spike(1400, 3, v); rotate(-20, 0.8, 1, v); hold(3, v); }
static void s_plop_chair(int v) {            /* dropping heavily into a chair: 2.0 g, small lean */
    rotate(15, 0.4, 0.7, v); spike(2000, 2, v); rotate(-12, 0.6, 1, v); hold(3, v); }
static void s_bend(int v) {                  /* medium-speed bend to pick something up and stand up */
    rotate(70, 1.0, 1, v); hold(1, v); rotate(-70, 1.0, 1, v); hold(3, v); }
static void s_slow_lower(int v) {            /* slow controlled lowering to lying (e.g. lying down on bed) */
    rotate(90, 3.0, 0.95, v); spike(1300, 2, v); hold(4, v); }
static void s_light_shake(int v) {           /* light shaking 4 Hz: +-0.8 g, +-80 dps for 4 s */
    for (double s = 0; s < 4; s += DT) feed(th + 0.1 * sin(2 * M_PI * 4 * s), 80 * cos(2 * M_PI * 4 * s), 1, 800 * sin(2 * M_PI * 4 * s), 0, 0, 30 * sin(2*M_PI*3*s), 0, v);
    hold(3, v); }
static void s_hard_shake(int v) {            /* vigorous shaking 5 Hz: +-2.5 g, +-250 dps for 3 s */
    for (double s = 0; s < 3; s += DT) feed(th + 0.4 * sin(2 * M_PI * 5 * s), 250 * cos(2 * M_PI * 5 * s), 1, 2500 * sin(2 * M_PI * 5 * s), 600 * sin(2*M_PI*7*s), 0, 60 * sin(2*M_PI*3*s), 0, v);
    hold(3, v); }
static void s_walk(int v) {                  /* walking 2 Hz: vertical +-0.35 g, sway +-15 dps, 10 s */
    for (double s = 0; s < 10; s += DT) feed(th, 15 * sin(2 * M_PI * 1 * s), 1, 100 * sin(2 * M_PI * 1 * s), 350 * sin(2 * M_PI * 2 * s), 0, 10 * sin(2*M_PI*2*s), 0, v);
    hold(2, v); }
static void s_jump(int v) {                  /* small hop: 0.2 s free fall, 1.5 g landing, upright */
    for (int i = 0; i < 10; i++) { feed(th, 0, 0.05, 0, 0, 0, 0, 0, v); }
    spike(1500, 3, v); hold(3, v); }
static void s_stumble_catch(int v) {        /* stumble: 35deg lurch with brief free fall, 1.6 g catch step, back upright */
    rotate(35, 0.3, 0.3, v); spike(1600, 2, v); rotate(-35, 0.6, 1, v); hold(3, v); }
static void s_drop_soft_flat(int v) {       /* very soft fall onto bed: 1.8 g, 90deg */
    rotate(90, 0.5, 0.35, v); spike(1800, 2, v); hold(4, v); }
static void s_put_on_table(int v) {          /* placing the device on a table: 90deg over 0.7 s, 1.3 g tap */
    rotate(90, 0.7, 0.95, v); spike(1300, 1, v); hold(4, v); }
static void s_carry_swing(int v) {           /* arm swinging while carrying: +-40deg at 1 Hz, 10 s */
    double base = th;
    for (double s = 0; s < 10; s += DT) { double a = 0.7 * sin(2 * M_PI * 1 * s); feed(base + a, 0.7 * 2 * M_PI * cos(2 * M_PI * s) * 180 / M_PI, 1, 0, 300 * sin(4*M_PI*s), 0, 0, 0, v); }
    hold(2, v); }

int main(void)
{
    int pass = 0, total = 0;
    printf("== True-positive cases ==\n");
    total++; pass += run("Forward fall with free fall + 3.5 g impact", s_fall_freefall, 1, 20);
    total++; pass += run("Sideways slump, no free fall, 2.8 g impact", s_fall_no_freefall, 1, 20);
    total++; pass += run("Backward fall onto cushion, 2.4 g impact", s_fall_backward_soft, 1, 20);
    total++; pass += run("Very soft fall onto bed, 1.8 g", s_drop_soft_flat, 1, 20);
    printf("== Normal activity / near-fall cases ==\n");
    total++; pass += run("Standing still", s_still, 0, 5);
    total++; pass += run("Walking", s_walk, 0, 10);
    total++; pass += run("Carrying (arm swing)", s_carry_swing, 0, 10);
    total++; pass += run("Sitting down", s_sit_down, 0, 20);
    total++; pass += run("Plopping heavily into chair (2 g)", s_plop_chair, 0, 20);
    total++; pass += run("Medium-speed bend and return", s_bend, 0, 20);
    total++; pass += run("Slow controlled lowering to lying", s_slow_lower, 0, 20);
    total++; pass += run("Light shaking", s_light_shake, 0, 20);
    total++; pass += run("Vigorous shaking", s_hard_shake, 0, 20);
    total++; pass += run("Small hop", s_jump, 0, 20);
    total++; pass += run("Stumble with free fall, caught (1.6 g)", s_stumble_catch, 0, 20);
    total++; pass += run("Placing device on table", s_put_on_table, 0, 20);

    printf("== Post-fall behaviour ==\n");
    srand(5); reset(); s_fall_freefall(0); hold(31, 1);
    printf("  long lie after 30 s: %s\n", fd.state == FD_STATE_LONG_LIE ? "OK" : "FAIL");
    total++; pass += (fd.state == FD_STATE_LONG_LIE);
    fd_event_t e = fd_button_short_press(&fd, t_ms);
    printf("  button ack clears long lie: %s\n", (e == FD_EVENT_ACKNOWLEDGED && fd.state == FD_STATE_MONITORING) ? "OK" : "FAIL");
    total++; pass += (e == FD_EVENT_ACKNOWLEDGED && fd.state == FD_STATE_MONITORING);

    srand(6); reset(); s_fall_freefall(0); hold(1, 0); rotate(-90, 1.0, 1, 1); hold(4, 1);
    printf("  self-recovery after getting up: %s\n", fd.state == FD_STATE_MONITORING ? "OK" : "FAIL");
    total++; pass += (fd.state == FD_STATE_MONITORING);

    srand(7); reset(); s_fall_freefall(0); hold(5, 0);
    printf("  fall indication stays latched while lying (5 s): %s\n", fd.state == FD_STATE_FALL_DETECTED ? "OK" : "FAIL");
    total++; pass += (fd.state == FD_STATE_FALL_DETECTED);

    srand(8); reset(); e = fd_button_long_press(&fd, t_ms); hold(3, 0);
    int sos_ok = (e == FD_EVENT_SOS && fd.state == FD_STATE_SOS);
    e = fd_button_short_press(&fd, t_ms);
    printf("  SOS via long press, cleared by short press: %s\n", (sos_ok && e == FD_EVENT_ACKNOWLEDGED) ? "OK" : "FAIL");
    total++; pass += (sos_ok && e == FD_EVENT_ACKNOWLEDGED);

    printf("\nSCENARIOS PASSED: %d / %d\n", pass, total);
    return pass != total;
}

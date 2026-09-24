/******************************************************************************
 * @file           : fall_detector.h
 * @brief          : CG2028 Assignment - multi-stage fall detection state machine
 *
 * The detector only ever sees EWMA-filtered sensor data (produced by the
 * assembly routine ewma_filter). It turns the six filtered axes into three
 * physical features and runs a state machine on them:
 *
 *   |a|    acceleration magnitude            [mg]   (1000 mg = 1 g at rest)
 *   |w|    angular-velocity magnitude        [dps]  (degrees per second)
 *   tilt   angle between the current gravity
 *          direction and the pre-event posture [deg]
 *
 *   CALIBRATING -> MONITORING -> FALL_SUSPECTED -> CONFIRMING -> FALL_DETECTED
 *                      ^              |                |              |
 *                      +--- rejected (near-fall) ------+              v
 *                      +--- acknowledged / recovered ------------ LONG_LIE
 *
 * SOS is a separate alert state entered by a long press of the user button.
 ******************************************************************************/

#ifndef FALL_DETECTOR_H
#define FALL_DETECTOR_H

#include <stdint.h>

/*------------------------------ Timing --------------------------------------*/
/* Sensor ODR is 52 Hz, so the main loop samples every 20 ms (50 Hz). */
#define FD_SAMPLE_PERIOD_MS          20U

/*--------------------------- Thresholds -------------------------------------
 * All values are in physical units and were chosen from the physics of a fall
 * and first trials; re-tune them with your own board (see README / UART log).
 *---------------------------------------------------------------------------*/

/* Calibration: ignore the first 300 ms while the EWMA outputs rise from their
 * initial value of 0, then average 700 ms of data as the reference posture. */
#define FD_CALIB_SETTLE_MS          300U
#define FD_CALIB_TOTAL_MS          1000U

/* Stage 1a - free fall. In free fall the accelerometer reads ~0 g. Normal
 * walking or sitting never drops |a| below ~0.7 g, so 0.6 g held for 60 ms
 * (3 samples) marks a genuine loss of support. */
#define FD_FREEFALL_MG              600
#define FD_FREEFALL_MIN_SAMPLES       3U

/* Stage 1b / 2 - impact. A fall onto the floor gives a short spike of 2-6 g.
 * Sitting down onto a chair stays around 1.2-1.5 g. The EWMA (alpha 0.5)
 * roughly halves a one-sample spike, so 1.7 g filtered is a clear impact.
 * Without a preceding free fall this is the only trigger, so it is strict.
 * After a confirmed free fall any landing above 1.3 g counts, so that a
 * soft landing (carpet, cushion, bed) is still recognised. */
#define FD_IMPACT_MG               1700
#define FD_IMPACT_AFTER_FREEFALL_MG 1300

/* After a free fall, the impact must arrive within 1 s. A human fall from
 * standing height lasts about 0.5-1.0 s. */
#define FD_IMPACT_WINDOW_MS        1000U

/* Stage 3 - rotation. A body falling over rotates about 90 degrees in well
 * under a second (peak >150 dps). Sitting down or bending at medium speed
 * rarely exceeds ~100 dps. */
#define FD_ROTATION_DPS             150

/* Stage 4 - posture change. Pre-event and post-event gravity directions must
 * differ by at least 45 degrees (e.g. upright -> lying). */
#define FD_POSTURE_CHANGE_DEG        45

/* Stage 5 - post-impact stillness. Wait 0.5 s for bounces to die out, then
 * observe 1.5 s; at least 70 % of samples must be "still":
 * |a| within 1 g +/- 0.15 g and |w| below 25 dps. Light shaking or carrying
 * keeps moving, so it fails this check. */
#define FD_POST_IMPACT_SETTLE_MS    500U
#define FD_STILL_OBSERVE_MS        1500U
#define FD_STILL_ACC_TOL_MG         150
#define FD_STILL_GYRO_DPS            25
#define FD_STILL_RATIO_PCT           70U

/* Recovery: device held back near the pre-fall posture (within 30 deg) and
 * steady for 3 s means the user got up. */
#define FD_RECOVERY_TILT_DEG         30
#define FD_RECOVERY_HOLD_MS        3000U

/* Long-lie escalation: still on the floor 30 s after a confirmed fall. */
#define FD_LONG_LIE_MS            30000U

/* After an alert is cleared, ignore new triggers for 2 s so that picking the
 * board back up does not immediately re-trigger. */
#define FD_REARM_DELAY_MS          2000U

/*------------------------------ Types ---------------------------------------*/
typedef enum
{
    FD_STATE_CALIBRATING = 0,
    FD_STATE_MONITORING,
    FD_STATE_FALL_SUSPECTED,
    FD_STATE_CONFIRMING,
    FD_STATE_FALL_DETECTED,
    FD_STATE_LONG_LIE,
    FD_STATE_SOS
} fd_state_t;

typedef enum
{
    FD_EVENT_NONE = 0,
    FD_EVENT_CALIBRATED,
    FD_EVENT_FREEFALL,          /* stage 1a: free fall seen, waiting for impact */
    FD_EVENT_IMPACT,            /* impact seen, checking rotation/posture/stillness */
    FD_EVENT_FALL_CONFIRMED,
    FD_EVENT_NEAR_FALL_REJECTED,
    FD_EVENT_RECOVERED,
    FD_EVENT_ACKNOWLEDGED,
    FD_EVENT_LONG_LIE,
    FD_EVENT_SOS
} fd_event_t;

/* Features computed from the filtered data on every sample. */
typedef struct
{
    int acc_mag_mg;
    int gyro_mag_dps;
    int tilt_deg;
} fd_features_t;

typedef struct
{
    fd_state_t    state;
    fd_features_t feat;

    /* Reference ("pre-event") posture: unit-free gravity vector in mg. */
    float ref[3];

    uint32_t state_entry_ms;
    uint32_t rearm_until_ms;

    /* Calibration accumulators */
    int32_t  calib_sum[3];
    uint32_t calib_count;

    /* Event evidence */
    uint32_t freefall_count;
    uint32_t impact_ms;
    uint32_t fall_ms;
    int      peak_acc_mg;
    int      peak_gyro_dps;
    int32_t  post_sum[3];      /* mean posture during the stillness window */
    uint32_t post_samples;
    uint32_t still_samples;
    uint32_t recovery_start_ms;
    int      recovery_active;

    /* Result of the last evaluation, for UART reporting */
    int  last_rotation_dps;
    int  last_posture_deg;
    int  last_still_pct;
    char last_reason[64];
} fall_detector_t;

/*------------------------------- API ----------------------------------------*/
void        fd_init(fall_detector_t *fd, uint32_t now_ms);
fd_event_t  fd_update(fall_detector_t *fd, const int acc_mg[3],
                      const int gyro_mdps[3], uint32_t now_ms);
fd_event_t  fd_button_short_press(fall_detector_t *fd, uint32_t now_ms);
fd_event_t  fd_button_long_press(fall_detector_t *fd, uint32_t now_ms);

int         fd_is_alerting(const fall_detector_t *fd);
const char *fd_state_name(fd_state_t state);

#endif /* FALL_DETECTOR_H */

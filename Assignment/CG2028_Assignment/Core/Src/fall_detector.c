/******************************************************************************
 * @file           : fall_detector.c
 * @brief          : CG2028 Assignment - multi-stage fall detection state machine
 *
 * Detection pipeline (all inputs are EWMA-filtered by the assembly routine):
 *
 *   1. Trigger   : free fall (|a| < 0.6 g for >= 60 ms)  -> wait up to 1 s
 *                  or a direct impact (|a| >= 1.7 g)       for an impact
 *   2. Impact    : |a| >= 1.3 g after a free fall (>= 1.7 g without one)
 *   3. Rotation  : peak |w| >= 150 dps in the second before / during the event
 *   4. Posture   : gravity direction changed by >= 45 deg vs the pre-event
 *                  posture
 *   5. Stillness : >= 70 % of samples still during 0.5 s .. 2.0 s after impact
 *
 * A fall is confirmed only if stages 2-5 all pass. Anything that triggers but
 * fails a later stage is reported as a rejected near-fall, together with the
 * reason, so that thresholds can be justified from the UART log.
 *
 * No maths library is needed: magnitudes use an integer square root and the
 * tilt angle uses a polynomial arccos approximation.
 ******************************************************************************/

#include "fall_detector.h"

#include <string.h>

#define GYRO_HISTORY_LEN   (1000U / FD_SAMPLE_PERIOD_MS)   /* 1 s of samples */

/* Reference posture follows slow, still posture changes (tau ~ 0.4 s). */
#define REF_TRACK_GAIN     0.05f

static int gyro_history[GYRO_HISTORY_LEN];
static uint32_t gyro_history_idx;

/*--------------------------- Maths helpers ----------------------------------*/

/* Integer square root (bit-by-bit method). */
static uint32_t isqrt_u32(uint32_t value)
{
    uint32_t result = 0;
    uint32_t bit = 1UL << 30;

    while (bit > value)
    {
        bit >>= 2;
    }
    while (bit != 0)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

/* Square root by Newton's method; accurate to <0.1 % for the ranges used. */
static float sqrt_positive(float x)
{
    if (x <= 0.0f)
    {
        return 0.0f;
    }
    float guess = (x > 1.0f) ? x * 0.5f : 1.0f;
    for (int i = 0; i < 20; i++)
    {
        guess = 0.5f * (guess + x / guess);
    }
    return guess;
}

/* arccos in degrees, Abramowitz & Stegun 4.4.45 (error < 0.01 deg). */
static int acos_deg(float x)
{
    const float rad_to_deg = 57.29578f;
    int negative = (x < 0.0f);

    if (negative)
    {
        x = -x;
    }
    if (x > 1.0f)
    {
        x = 1.0f;
    }
    float poly = 1.5707288f + x * (-0.2121144f + x * (0.0742610f - 0.0187293f * x));
    float angle = sqrt_positive(1.0f - x) * poly;
    if (negative)
    {
        angle = 3.14159265f - angle;
    }
    return (int)(angle * rad_to_deg + 0.5f);
}

/* Angle between two 3-D vectors in degrees (0 if either is ~zero). */
static int angle_between_deg(const float a[3], const float b[3])
{
    float dot   = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    float norm2 = (a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
                * (b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    if (norm2 < 1.0f)
    {
        return 0;
    }
    return acos_deg(dot / sqrt_positive(norm2));
}

static int abs_int(int v)
{
    return (v < 0) ? -v : v;
}

/*--------------------------- Internal helpers -------------------------------*/

static int is_still(const fd_features_t *feat)
{
    return (abs_int(feat->acc_mag_mg - 1000) < FD_STILL_ACC_TOL_MG)
        && (feat->gyro_mag_dps < FD_STILL_GYRO_DPS);
}

static int recent_peak_gyro(void)
{
    int peak = 0;
    for (uint32_t i = 0; i < GYRO_HISTORY_LEN; i++)
    {
        if (gyro_history[i] > peak)
        {
            peak = gyro_history[i];
        }
    }
    return peak;
}

static void enter_state(fall_detector_t *fd, fd_state_t state, uint32_t now_ms)
{
    fd->state = state;
    fd->state_entry_ms = now_ms;
}

/* Start collecting evidence for a possible fall. The rotation peak includes
 * the last second so that rotation before the trigger is not lost. */
static void start_event(fall_detector_t *fd, uint32_t now_ms)
{
    fd->freefall_count = 0;
    fd->peak_acc_mg    = fd->feat.acc_mag_mg;
    fd->peak_gyro_dps  = recent_peak_gyro();
    fd->post_sum[0] = fd->post_sum[1] = fd->post_sum[2] = 0;
    fd->post_samples  = 0;
    fd->still_samples = 0;
    fd->last_reason[0] = '\0';
    (void)now_ms;
}

static void begin_confirming(fall_detector_t *fd, uint32_t now_ms)
{
    fd->impact_ms = now_ms;
    enter_state(fd, FD_STATE_CONFIRMING, now_ms);
}

static void return_to_monitoring(fall_detector_t *fd, uint32_t now_ms,
                                 uint32_t rearm_delay_ms)
{
    fd->freefall_count  = 0;
    fd->recovery_active = 0;
    fd->rearm_until_ms  = now_ms + rearm_delay_ms;
    enter_state(fd, FD_STATE_MONITORING, now_ms);
}

static void append_reason(char *buf, const char *text)
{
    size_t used = strlen(buf);
    size_t room = 64U - used - 1U;

    if (used > 0 && room > 2)
    {
        strncat(buf, ", ", room);
        room -= 2;
    }
    strncat(buf, text, room);
}

/* Decide between a real fall and a near-fall at the end of the stillness
 * window. Every condition uses a different physical quantity. */
static fd_event_t evaluate_event(fall_detector_t *fd, uint32_t now_ms)
{
    float mean_posture[3] = {0.0f, 0.0f, 0.0f};

    if (fd->post_samples > 0)
    {
        for (int axis = 0; axis < 3; axis++)
        {
            mean_posture[axis] = (float)fd->post_sum[axis] / (float)fd->post_samples;
        }
    }

    fd->last_rotation_dps = fd->peak_gyro_dps;
    fd->last_posture_deg  = angle_between_deg(mean_posture, fd->ref);
    fd->last_still_pct    = (fd->post_samples > 0)
                          ? (int)((fd->still_samples * 100U) / fd->post_samples)
                          : 0;

    int rotation_ok = (fd->last_rotation_dps >= FD_ROTATION_DPS);
    int posture_ok  = (fd->last_posture_deg  >= FD_POSTURE_CHANGE_DEG);
    int still_ok    = (fd->last_still_pct    >= (int)FD_STILL_RATIO_PCT);

    if (rotation_ok && posture_ok && still_ok)
    {
        fd->fall_ms = now_ms;
        fd->recovery_active = 0;
        enter_state(fd, FD_STATE_FALL_DETECTED, now_ms);
        return FD_EVENT_FALL_CONFIRMED;
    }

    fd->last_reason[0] = '\0';
    if (!rotation_ok)
    {
        append_reason(fd->last_reason, "rotation too slow");
    }
    if (!posture_ok)
    {
        append_reason(fd->last_reason, "no posture change");
    }
    if (!still_ok)
    {
        append_reason(fd->last_reason, "still moving after impact");
    }
    return_to_monitoring(fd, now_ms, 0);
    return FD_EVENT_NEAR_FALL_REJECTED;
}

static void compute_features(fall_detector_t *fd, const int acc_mg[3],
                             const int gyro_mdps[3])
{
    uint32_t acc_sq = 0;
    uint32_t gyro_sq = 0;

    for (int axis = 0; axis < 3; axis++)
    {
        int a = acc_mg[axis];
        int g = gyro_mdps[axis] / 1000;           /* mdps -> dps */
        acc_sq  += (uint32_t)(a * a);
        gyro_sq += (uint32_t)(g * g);
    }
    fd->feat.acc_mag_mg   = (int)isqrt_u32(acc_sq);
    fd->feat.gyro_mag_dps = (int)isqrt_u32(gyro_sq);

    float acc_vec[3] = {(float)acc_mg[0], (float)acc_mg[1], (float)acc_mg[2]};
    fd->feat.tilt_deg = angle_between_deg(acc_vec, fd->ref);

    gyro_history[gyro_history_idx] = fd->feat.gyro_mag_dps;
    gyro_history_idx = (gyro_history_idx + 1U) % GYRO_HISTORY_LEN;
}

/*------------------------------- API ----------------------------------------*/

void fd_init(fall_detector_t *fd, uint32_t now_ms)
{
    memset(fd, 0, sizeof(*fd));
    memset(gyro_history, 0, sizeof(gyro_history));
    gyro_history_idx = 0;
    enter_state(fd, FD_STATE_CALIBRATING, now_ms);
}

fd_event_t fd_update(fall_detector_t *fd, const int acc_mg[3],
                     const int gyro_mdps[3], uint32_t now_ms)
{
    compute_features(fd, acc_mg, gyro_mdps);

    const fd_features_t *feat = &fd->feat;
    uint32_t in_state_ms = now_ms - fd->state_entry_ms;

    switch (fd->state)
    {
    case FD_STATE_CALIBRATING:
        if (in_state_ms >= FD_CALIB_SETTLE_MS)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                fd->calib_sum[axis] += acc_mg[axis];
            }
            fd->calib_count++;
        }
        if (in_state_ms >= FD_CALIB_TOTAL_MS && fd->calib_count > 0)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                fd->ref[axis] = (float)fd->calib_sum[axis] / (float)fd->calib_count;
            }
            return_to_monitoring(fd, now_ms, 0);
            return FD_EVENT_CALIBRATED;
        }
        break;

    case FD_STATE_MONITORING:
        /* Track the wearer's normal posture while they are still. */
        if (is_still(feat))
        {
            for (int axis = 0; axis < 3; axis++)
            {
                fd->ref[axis] += REF_TRACK_GAIN * ((float)acc_mg[axis] - fd->ref[axis]);
            }
        }

        if ((int32_t)(now_ms - fd->rearm_until_ms) < 0)
        {
            fd->freefall_count = 0;
            break;
        }

        fd->freefall_count = (feat->acc_mag_mg < FD_FREEFALL_MG)
                           ? fd->freefall_count + 1U : 0U;

        if (fd->freefall_count >= FD_FREEFALL_MIN_SAMPLES)
        {
            start_event(fd, now_ms);
            enter_state(fd, FD_STATE_FALL_SUSPECTED, now_ms);
            return FD_EVENT_FREEFALL;
        }
        if (feat->acc_mag_mg >= FD_IMPACT_MG)
        {
            start_event(fd, now_ms);
            begin_confirming(fd, now_ms);
            return FD_EVENT_IMPACT;
        }
        break;

    case FD_STATE_FALL_SUSPECTED:
        if (feat->gyro_mag_dps > fd->peak_gyro_dps)
        {
            fd->peak_gyro_dps = feat->gyro_mag_dps;
        }
        if (feat->acc_mag_mg >= FD_IMPACT_AFTER_FREEFALL_MG)
        {
            fd->peak_acc_mg = feat->acc_mag_mg;
            begin_confirming(fd, now_ms);
            return FD_EVENT_IMPACT;
        }
        if (in_state_ms >= FD_IMPACT_WINDOW_MS)
        {
            strcpy(fd->last_reason, "free fall but no impact");
            fd->last_rotation_dps = fd->peak_gyro_dps;
            fd->last_posture_deg  = feat->tilt_deg;
            fd->last_still_pct    = 0;
            return_to_monitoring(fd, now_ms, 0);
            return FD_EVENT_NEAR_FALL_REJECTED;
        }
        break;

    case FD_STATE_CONFIRMING:
    {
        uint32_t since_impact = now_ms - fd->impact_ms;

        if (feat->acc_mag_mg > fd->peak_acc_mg)
        {
            fd->peak_acc_mg = feat->acc_mag_mg;
        }
        if (since_impact < FD_POST_IMPACT_SETTLE_MS)
        {
            /* Rotation often continues briefly after first contact. */
            if (feat->gyro_mag_dps > fd->peak_gyro_dps)
            {
                fd->peak_gyro_dps = feat->gyro_mag_dps;
            }
        }
        else
        {
            for (int axis = 0; axis < 3; axis++)
            {
                fd->post_sum[axis] += acc_mg[axis];
            }
            fd->post_samples++;
            if (is_still(feat))
            {
                fd->still_samples++;
            }
        }
        if (since_impact >= FD_POST_IMPACT_SETTLE_MS + FD_STILL_OBSERVE_MS)
        {
            return evaluate_event(fd, now_ms);
        }
        break;
    }

    case FD_STATE_FALL_DETECTED:
        if (now_ms - fd->fall_ms >= FD_LONG_LIE_MS)
        {
            enter_state(fd, FD_STATE_LONG_LIE, now_ms);
            return FD_EVENT_LONG_LIE;
        }
        /* Self-recovery: back near the pre-fall posture and steady for 3 s. */
        if (feat->tilt_deg < FD_RECOVERY_TILT_DEG && is_still(feat))
        {
            if (!fd->recovery_active)
            {
                fd->recovery_active = 1;
                fd->recovery_start_ms = now_ms;
            }
            else if (now_ms - fd->recovery_start_ms >= FD_RECOVERY_HOLD_MS)
            {
                return_to_monitoring(fd, now_ms, FD_REARM_DELAY_MS);
                return FD_EVENT_RECOVERED;
            }
        }
        else
        {
            fd->recovery_active = 0;
        }
        break;

    case FD_STATE_LONG_LIE:
    case FD_STATE_SOS:
    default:
        /* Emergency states are cleared only by the user button. */
        break;
    }

    return FD_EVENT_NONE;
}

int fd_is_alerting(const fall_detector_t *fd)
{
    return (fd->state == FD_STATE_FALL_DETECTED)
        || (fd->state == FD_STATE_LONG_LIE)
        || (fd->state == FD_STATE_SOS);
}

/* Short press = "I am OK": acknowledges any alert. */
fd_event_t fd_button_short_press(fall_detector_t *fd, uint32_t now_ms)
{
    if (fd_is_alerting(fd))
    {
        return_to_monitoring(fd, now_ms, FD_REARM_DELAY_MS);
        return FD_EVENT_ACKNOWLEDGED;
    }
    return FD_EVENT_NONE;
}

/* Long press = manual SOS (or acknowledge if an alert is already active). */
fd_event_t fd_button_long_press(fall_detector_t *fd, uint32_t now_ms)
{
    if (fd_is_alerting(fd))
    {
        return fd_button_short_press(fd, now_ms);
    }
    enter_state(fd, FD_STATE_SOS, now_ms);
    return FD_EVENT_SOS;
}

const char *fd_state_name(fd_state_t state)
{
    switch (state)
    {
    case FD_STATE_CALIBRATING:    return "CALIBRATING";
    case FD_STATE_MONITORING:     return "NORMAL";
    case FD_STATE_FALL_SUSPECTED: return "FREE-FALL?";
    case FD_STATE_CONFIRMING:     return "CONFIRMING";
    case FD_STATE_FALL_DETECTED:  return "FALL DETECTED";
    case FD_STATE_LONG_LIE:       return "LONG LIE";
    case FD_STATE_SOS:            return "SOS";
    default:                      return "UNKNOWN";
    }
}

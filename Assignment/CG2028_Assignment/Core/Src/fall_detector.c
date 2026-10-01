/******************************************************************************
 * @file           : fall_detector.c
 * @brief          : CG2028 Assignment - fall detection for the ElderCare wearable
 *
 * See fall_detector.h for the phase machine and the meaning of each threshold.
 * Only EWMA-filtered data enters this module; it never touches the sensors or
 * the HAL, so the same code can be exercised by the PC simulation in
 * Part2_Simulation/.
 ******************************************************************************/

#include "fall_detector.h"

#include <string.h>

/*--------------------------- Maths helpers ----------------------------------*/

static uint64_t square(int64_t input)
{
	return (uint64_t)(input * input);
}

static uint64_t square_vector_magnitude(const int vector[3])
{
	int64_t x = vector[0];
	int64_t y = vector[1];
	int64_t z = vector[2];

	return (uint64_t)((x * x) + (y * y) + (z * z));
}

static bool is_vector_change_exceeded(const int curr_vector[3],
                                      const int prev_vector[3],
                                      uint32_t threshold)
{
	uint64_t delta = 0;

	for (int i = 0; i < 3; i++) {
		delta += square((int64_t)curr_vector[i] - prev_vector[i]);
	}

	return delta > square(threshold);
}

/* Integer square root, used only to report magnitudes over UART. */
static uint32_t isqrt_u64(uint64_t value)
{
	uint64_t result = 0;
	uint64_t bit = 1ULL << 62;

	while (bit > value) {
		bit >>= 2;
	}
	while (bit != 0) {
		if (value >= result + bit) {
			value -= result + bit;
			result = (result >> 1) + bit;
		} else {
			result >>= 1;
		}
		bit >>= 2;
	}
	return (uint32_t)result;
}

/* Square root by Newton's method; avoids a dependency on the maths library. */
static float sqrt_positive(float x)
{
	if (x <= 0.0f) {
		return 0.0f;
	}
	float guess = (x > 1.0f) ? x * 0.5f : 1.0f;
	for (int i = 0; i < 20; i++) {
		guess = 0.5f * (guess + x / guess);
	}
	return guess;
}

/* arccos in degrees, Abramowitz & Stegun 4.4.45 (error < 0.01 deg). */
static int acos_deg(float x)
{
	const float rad_to_deg = 57.29578f;
	bool negative = (x < 0.0f);

	if (negative) {
		x = -x;
	}
	if (x > 1.0f) {
		x = 1.0f;
	}
	float poly = 1.5707288f + x * (-0.2121144f + x * (0.0742610f - 0.0187293f * x));
	float angle = sqrt_positive(1.0f - x) * poly;
	if (negative) {
		angle = 3.14159265f - angle;
	}
	return (int)(angle * rad_to_deg + 0.5f);
}

/* Angle between two acceleration vectors, i.e. how much the posture changed. */
static int angle_between_deg(const int a[3], const int b[3])
{
	float dot = 0.0f;

	for (int i = 0; i < 3; i++) {
		dot += (float)a[i] * (float)b[i];
	}

	float norm = (float)isqrt_u64(square_vector_magnitude(a)) * (float)isqrt_u64(square_vector_magnitude(b));
	if (norm < 1.0f) {
		return 0;
	}
	return acos_deg(dot / norm);
}

/*--------------------------- Internal helpers -------------------------------*/

static void enter_phase(FallDetector *detector, FallPhase phase, uint32_t now_ms)
{
	detector->phase = phase;
	detector->phase_start_ms = now_ms;
}

/* Highest rotation seen in the last second. A collapse rotates the body
 * before it hits the floor, so that rotation must not be discarded when the
 * impact itself is what triggers the detector. */
static int recent_peak_gyro(const FallDetector *detector)
{
	int peak = 0;

	for (uint32_t i = 0; i < HISTORY_LEN; i++) {
		if (detector->gyro_history[i] > peak) {
			peak = detector->gyro_history[i];
		}
	}
	return peak;
}

static int recent_min_accel(const FallDetector *detector)
{
	int lowest = detector->accel_history[0];

	for (uint32_t i = 1; i < HISTORY_LEN; i++) {
		if (detector->accel_history[i] < lowest) {
			lowest = detector->accel_history[i];
		}
	}
	return lowest;
}

static uint32_t current_osc_ms(const FallDetector *detector, uint32_t now_ms)
{
	if ((now_ms - detector->last_reversal_ms) > OSCILLATION_GAP_MS) {
		return 0U;
	}
	return detector->last_reversal_ms - detector->osc_chain_start_ms;
}

/* Discard the posture samples collected so far after an impact. */
static void reset_post_window(FallDetector *detector)
{
	detector->post_sum[0] = detector->post_sum[1] = detector->post_sum[2] = 0;
	detector->post_samples = 0;
}

static void start_suspected_fall(FallDetector *detector, uint32_t now_ms)
{
	detector->free_fall_samples = 0;
	detector->peak_accel_mg = detector->accel_mag_mg;
	detector->peak_gyro_mdps = recent_peak_gyro(detector);
	detector->min_accel_mg = recent_min_accel(detector);
	detector->max_osc_ms = current_osc_ms(detector, now_ms);
	reset_post_window(detector);
	detector->is_inactive = false;
	detector->inactivity_start_ms = now_ms;
	detector->last_reason[0] = '\0';
}

static void back_to_monitoring(FallDetector *detector, uint32_t now_ms,
                               uint32_t rearm_delay_ms)
{
	detector->free_fall_samples = 0;
	detector->free_fall_seen = false;
	detector->is_recovering = false;
	detector->is_inactive = false;
	detector->rearm_until_ms = now_ms + rearm_delay_ms;
	enter_phase(detector, FALL_PHASE_MONITORING, now_ms);
}

/* Adds text to last_reason as a comma-separated list, truncating if full. */
static void append_reason(FallDetector *detector, const char *text)
{
	size_t used = strlen(detector->last_reason);
	size_t room = sizeof(detector->last_reason) - used - 1U;

	if (used > 0 && room > 2) {
		strncat(detector->last_reason, ", ", room);
		room -= 2;
	}
	strncat(detector->last_reason, text, room);
}

/* Decide between a real fall and a near-fall at the end of the observation
 * window. Each check uses a different physical quantity. */
static FallEvent evaluate_suspected_fall(FallDetector *detector, uint32_t now_ms)
{
	int mean_posture[3] = {0, 0, 0};

	if (detector->post_samples > 0) {
		for (int i = 0; i < 3; i++) {
			mean_posture[i] = (int)(detector->post_sum[i] / (int32_t)detector->post_samples);
		}
	}

	detector->last_rotation_mdps = detector->peak_gyro_mdps;
	detector->last_posture_deg = detector->has_pre_fall_accel
	                           ? angle_between_deg(mean_posture, detector->pre_fall_accel)
	                           : 0;
	detector->last_inactive_pct = detector->is_inactive ? 100 : 0;

	bool rotation_ok = (detector->peak_gyro_mdps >= RAPID_ROTATION_MDPS);
	bool oscillated = (detector->max_osc_ms >= OSCILLATION_MAX_MS);
	bool posture_ok = (detector->last_posture_deg >= POSTURE_CHANGE_DEG) && !oscillated;
	bool dropped = (detector->min_accel_mg < DEEP_FREE_FALL_MG);
	bool inactivity_ok = detector->is_inactive;

	if (rotation_ok && (posture_ok || dropped) && inactivity_ok) {
		detector->alarm_ms = now_ms;
		detector->is_recovering = false;
		detector->moved_since_alarm = false;
		enter_phase(detector, FALL_PHASE_ALARM, now_ms);
		return FALL_EVENT_FALL_CONFIRMED;
	}

	detector->last_reason[0] = '\0';
	if (!rotation_ok) {
		append_reason(detector, "rotation too slow");
	}
	if (!posture_ok && !dropped) {
		append_reason(detector, oscillated ? "shaken, no drop" : "no posture change or drop");
	}
	if (!inactivity_ok) {
		append_reason(detector, "still moving after impact");
	}
	back_to_monitoring(detector, now_ms, 0);
	return FALL_EVENT_NEAR_FALL;
}

/* Per-sample features and flags, in the units of the thresholds. */
static void update_features(FallDetector *detector, const int accel_mg[3],
                            const int gyro_mdps[3], uint32_t now_ms)
{
	uint64_t accel_mag_sqrd = square_vector_magnitude(accel_mg);
	uint64_t gyro_mag_sqrd = square_vector_magnitude(gyro_mdps);

	detector->accel_mag_mg = (int)isqrt_u64(accel_mag_sqrd);
	detector->gyro_mag_mdps = (int)isqrt_u64(gyro_mag_sqrd);

	detector->is_free_fall = (accel_mag_sqrd < square(FREE_FALL_MG));
	detector->is_rapid_rotation = (gyro_mag_sqrd > square(RAPID_ROTATION_MDPS));
	detector->is_stationary = (accel_mag_sqrd > square(STATIONARY_ACCEL_MIN_MG)) &&
	                          (accel_mag_sqrd < square(STATIONARY_ACCEL_MAX_MG)) &&
	                          (gyro_mag_sqrd < square(STATIONARY_ROTATION_MDPS));

	detector->tilt_deg = detector->has_pre_fall_accel
	                   ? angle_between_deg(accel_mg, detector->pre_fall_accel)
	                   : 0;

	detector->gyro_history[detector->history_idx] = detector->gyro_mag_mdps;
	detector->accel_history[detector->history_idx] = detector->accel_mag_mg;
	detector->history_idx = (detector->history_idx + 1U) % HISTORY_LEN;

	if (detector->gyro_mag_mdps > detector->peak_gyro_mdps) {
		detector->peak_gyro_mdps = detector->gyro_mag_mdps;
	}
	if (detector->accel_mag_mg > detector->peak_accel_mg) {
		detector->peak_accel_mg = detector->accel_mag_mg;
	}
	if (detector->accel_mag_mg < detector->min_accel_mg) {
		detector->min_accel_mg = detector->accel_mag_mg;
	}

	for (int i = 0; i < 3; i++) {
		int sign = (gyro_mdps[i] > OSCILLATION_MDPS) ? 1 : (gyro_mdps[i] < -OSCILLATION_MDPS) ? -1 : 0;

		if (sign == 0) {
			continue;
		}
		if (detector->last_gyro_sign[i] == -sign &&
		    (now_ms - detector->last_gyro_sign_ms[i]) <= OSCILLATION_GAP_MS) {
			if ((now_ms - detector->last_reversal_ms) > OSCILLATION_GAP_MS) {
				detector->osc_chain_start_ms = now_ms;
			}
			detector->last_reversal_ms = now_ms;
		}
		detector->last_gyro_sign[i] = sign;
		detector->last_gyro_sign_ms[i] = now_ms;
	}

	uint32_t osc_ms = current_osc_ms(detector, now_ms);
	if (osc_ms > detector->max_osc_ms) {
		detector->max_osc_ms = osc_ms;
	}
}

/*--------------------------- Phase handlers ---------------------------------*/

/* The most recent stationary posture is the reference that a fall is measured
 * against, so it follows slow, deliberate posture changes. Returns READY the
 * first time a reference is set. */
static FallEvent update_reference_posture(FallDetector *detector,
                                          const int accel_mg[3], uint32_t now_ms)
{
	if (!detector->is_stationary) {
		detector->reference_still_start_ms = now_ms;
		return FALL_EVENT_NONE;
	}
	if ((now_ms - detector->reference_still_start_ms) < REFERENCE_HOLD_MS) {
		return FALL_EVENT_NONE;
	}

	for (int i = 0; i < 3; i++) {
		detector->pre_fall_accel[i] = accel_mg[i];
	}
	if (!detector->has_pre_fall_accel) {
		detector->has_pre_fall_accel = true;
		return FALL_EVENT_READY;
	}
	return FALL_EVENT_NONE;
}

/* MONITORING: look for any of the three ways a fall can start. */
static FallEvent update_monitoring(FallDetector *detector, const int accel_mg[3],
                                   uint32_t now_ms)
{
	FallEvent event = update_reference_posture(detector, accel_mg, now_ms);
	bool warmed_up = (now_ms - detector->startup_timer_ms) >= STARTUP_WARMUP_MS;
	bool rearmed = (int32_t)(now_ms - detector->rearm_until_ms) >= 0;

	if (!warmed_up || !rearmed) {
		detector->free_fall_samples = 0;
		return event;
	}

	detector->free_fall_samples = detector->is_free_fall
	                            ? detector->free_fall_samples + 1U : 0U;

	if (detector->free_fall_samples >= FREE_FALL_MIN_SAMPLES) {
		start_suspected_fall(detector, now_ms);
		detector->free_fall_seen = true;
		enter_phase(detector, FALL_PHASE_AWAIT_IMPACT, now_ms);
		event = FALL_EVENT_FREE_FALL;
	} else if (detector->accel_mag_mg >= IMPACT_MG) {
		/* Hard impact with no preceding free fall, e.g. a trip. */
		start_suspected_fall(detector, now_ms);
		detector->free_fall_seen = false;
		detector->impact_ms = now_ms;
		enter_phase(detector, FALL_PHASE_POST_IMPACT, now_ms);
		event = FALL_EVENT_IMPACT;
	} else if (detector->has_prev_accel && detector->is_rapid_rotation &&
	           is_vector_change_exceeded(accel_mg, detector->prev_accel,
	                                     SUDDEN_CHANGE_MG)) {
		/* Sudden jerk together with fast rotation: a collapse can look
		 * like this without ever reaching free fall. */
		start_suspected_fall(detector, now_ms);
		detector->free_fall_seen = false;
		enter_phase(detector, FALL_PHASE_AWAIT_IMPACT, now_ms);
		event = FALL_EVENT_FREE_FALL;
	}
	return event;
}

/* AWAIT_IMPACT: the body is falling; wait up to IMPACT_WINDOW_MS for it to
 * land. */
static FallEvent update_await_impact(FallDetector *detector, uint32_t in_phase_ms,
                                     uint32_t now_ms)
{
	/* After a confirmed free fall even a soft landing counts, because the
	 * loss of support has already been established. */
	int impact_threshold = detector->free_fall_seen ? SOFT_IMPACT_MG : IMPACT_MG;

	if (detector->min_accel_mg < DEEP_FREE_FALL_MG) {
		impact_threshold = LANDING_MG;
	}

	if (detector->accel_mag_mg >= impact_threshold) {
		detector->impact_ms = now_ms;
		enter_phase(detector, FALL_PHASE_POST_IMPACT, now_ms);
		return FALL_EVENT_IMPACT;
	}
	if (in_phase_ms >= IMPACT_WINDOW_MS) {
		strcpy(detector->last_reason, "no impact after trigger");
		detector->last_rotation_mdps = detector->peak_gyro_mdps;
		detector->last_posture_deg = detector->tilt_deg;
		detector->last_inactive_pct = 0;
		back_to_monitoring(detector, now_ms, 0);
		return FALL_EVENT_NEAR_FALL;
	}
	return FALL_EVENT_NONE;
}

/* POST_IMPACT: after the settle time, wait for continuous stillness and then
 * decide. Gives up and decides anyway at the end of the window. */
static FallEvent update_post_impact(FallDetector *detector, const int accel_mg[3],
                                    uint32_t in_phase_ms, uint32_t now_ms)
{
	if (in_phase_ms >= POST_IMPACT_SETTLE_MS) {
		if (detector->is_stationary) {
			if (detector->post_samples == 0) {
				detector->inactivity_start_ms = now_ms;
			}
			for (int i = 0; i < 3; i++) {
				detector->post_sum[i] += accel_mg[i];
			}
			detector->post_samples++;

			if ((now_ms - detector->inactivity_start_ms) >= INACTIVITY_REQUIRED_MS) {
				detector->is_inactive = true;
				return evaluate_suspected_fall(detector, now_ms);
			}
		} else {
			reset_post_window(detector);
		}
	}
	if (in_phase_ms >= POST_IMPACT_WINDOW_MS) {
		return evaluate_suspected_fall(detector, now_ms);
	}
	return FALL_EVENT_NONE;
}

/* ALARM: escalate after LONG_LIE_MS, or clear once the wearer has got up
 * (moved away from the floor posture, then back near the reference and
 * steady for RECOVERY_HOLD_MS). */
static FallEvent update_alarm(FallDetector *detector, uint32_t now_ms)
{
	if ((now_ms - detector->alarm_ms) >= LONG_LIE_MS) {
		enter_phase(detector, FALL_PHASE_LONG_LIE, now_ms);
		return FALL_EVENT_LONG_LIE;
	}

	if (detector->tilt_deg >= RECOVERY_LEAVE_DEG) {
		detector->moved_since_alarm = true;
	}
	if (!detector->moved_since_alarm ||
	    detector->tilt_deg >= RECOVERY_TILT_DEG || !detector->is_stationary) {
		detector->is_recovering = false;
		return FALL_EVENT_NONE;
	}

	if (!detector->is_recovering) {
		detector->is_recovering = true;
		detector->recovery_start_ms = now_ms;
	} else if ((now_ms - detector->recovery_start_ms) >= RECOVERY_HOLD_MS) {
		back_to_monitoring(detector, now_ms, REARM_DELAY_MS);
		return FALL_EVENT_RECOVERED;
	}
	return FALL_EVENT_NONE;
}

/*------------------------------- API ----------------------------------------*/

void FallDetector_Init(FallDetector *detector, uint32_t now_ms)
{
	memset(detector, 0, sizeof(*detector));
	detector->phase = FALL_PHASE_MONITORING;
	detector->startup_timer_ms = now_ms;
	detector->phase_start_ms = now_ms;
	detector->reference_still_start_ms = now_ms;
	detector->last_reversal_ms = now_ms - OSCILLATION_GAP_MS - 1U;

	for (uint32_t i = 0; i < HISTORY_LEN; i++) {
		detector->accel_history[i] = 1000;
	}
}

FallEvent FallDetector_Update(FallDetector *detector, const int accel_mg[3],
                              const int gyro_mdps[3], uint32_t now_ms)
{
	update_features(detector, accel_mg, gyro_mdps, now_ms);

	uint32_t in_phase_ms = now_ms - detector->phase_start_ms;
	FallEvent event = FALL_EVENT_NONE;

	switch (detector->phase) {
	case FALL_PHASE_MONITORING:
		event = update_monitoring(detector, accel_mg, now_ms);
		break;
	case FALL_PHASE_AWAIT_IMPACT:
		event = update_await_impact(detector, in_phase_ms, now_ms);
		break;
	case FALL_PHASE_POST_IMPACT:
		event = update_post_impact(detector, accel_mg, in_phase_ms, now_ms);
		break;
	case FALL_PHASE_ALARM:
		event = update_alarm(detector, now_ms);
		break;
	case FALL_PHASE_LONG_LIE:
	case FALL_PHASE_SOS:
	default:
		/* Emergency phases are cleared by the user button only. */
		break;
	}

	for (int i = 0; i < 3; i++) {
		detector->prev_accel[i] = accel_mg[i];
	}
	detector->has_prev_accel = true;

	return event;
}

FallEvent FallDetector_Acknowledge(FallDetector *detector, uint32_t now_ms)
{
	if (FallDetector_IsAlarming(detector)) {
		back_to_monitoring(detector, now_ms, REARM_DELAY_MS);
		return FALL_EVENT_ACKNOWLEDGED;
	}
	return FALL_EVENT_NONE;
}

FallEvent FallDetector_RequestSOS(FallDetector *detector, uint32_t now_ms)
{
	if (detector->phase == FALL_PHASE_SOS) {
		return FALL_EVENT_NONE;
	}

	enter_phase(detector, FALL_PHASE_SOS, now_ms);
	return FALL_EVENT_SOS;
}

bool FallDetector_IsAlarming(const FallDetector *detector)
{
	return (detector->phase == FALL_PHASE_ALARM) ||
	       (detector->phase == FALL_PHASE_LONG_LIE) ||
	       (detector->phase == FALL_PHASE_SOS);
}

const char *FallDetector_PhaseName(FallPhase phase)
{
	switch (phase) {
	case FALL_PHASE_MONITORING:   return "NORMAL";
	case FALL_PHASE_AWAIT_IMPACT: return "FALLING?";
	case FALL_PHASE_POST_IMPACT:  return "CHECKING";
	case FALL_PHASE_ALARM:        return "FALL DETECTED";
	case FALL_PHASE_LONG_LIE:     return "LONG LIE";
	case FALL_PHASE_SOS:          return "SOS";
	default:                      return "UNKNOWN";
	}
}

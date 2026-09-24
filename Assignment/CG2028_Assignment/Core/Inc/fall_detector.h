/******************************************************************************
 * @file           : fall_detector.h
 * @brief          : CG2028 Assignment - fall detection for the ElderCare wearable
 *
 * Phase machine (all inputs are EWMA-filtered by the assembly routine):
 *
 *   MONITORING --free fall / sudden change--> AWAIT_IMPACT --impact--> POST_IMPACT
 *        ^                                         |                      |
 *        |                                    no impact                 checks
 *        +----------- near fall (reason reported) --+----------------------+
 *        |                                                          all pass |
 *        +--- acknowledged / recovered --- ALARM <---------------------------+
 *                                           | no response for LONG_LIE_MS
 *                                           v
 *                                        LONG_LIE        SOS (button, any time)
 *
 * Confirmation needs all three, each from a different physical quantity:
 *   1. rapid rotation   peak |w| >= RAPID_ROTATION_MDPS   (gyroscope)
 *   2. posture change   >= POSTURE_CHANGE_DEG vs the last stationary posture
 *   3. inactivity       stationary for >= INACTIVITY_REQUIRED_MS, and for
 *                       >= INACTIVITY_RATIO_PCT of the observation window
 *
 * Magnitude comparisons are done on squared values so that no square root is
 * needed. The reported tilt angle uses a polynomial arccos approximation, so
 * the maths library is not required either.
 ******************************************************************************/

#ifndef FALL_DETECTOR_H
#define FALL_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

/*--------------------------- Sampling ---------------------------------------*/
#define SAMPLE_PERIOD_MS           20   /* 50 Hz; sensor ODR is 52 Hz         */
#define STARTUP_WARMUP_MS         500   /* EWMA outputs start at 0; ignore    */
#define IMPACT_WINDOW_MS         1000   /* a fall reaches the floor within 1 s */
#define POST_IMPACT_WINDOW_MS    2000   /* observation window after impact    */
#define POST_IMPACT_SETTLE_MS     500   /* bounces/rotation may continue      */
#define INACTIVITY_REQUIRED_MS    750   /* continuous stillness needed        */
#define UART_REPORT_PERIOD_MS     200

/*--------------------------- Thresholds -------------------------------------
 * Units: acceleration in mg (1000 mg = 1 g), rotation in mdps (1000 mdps =
 * 1 dps). Tune these on the board using the UART log.
 *---------------------------------------------------------------------------*/
#define FREE_FALL_MG               650  /* < 0.65 g: support lost             */
#define FREE_FALL_MIN_SAMPLES        3  /* 60 ms, rejects single noisy sample */
#define IMPACT_MG                 1800  /* hard landing, no free fall needed  */
#define SOFT_IMPACT_MG            1300  /* enough after a confirmed free fall */
#define SUDDEN_CHANGE_MG           600  /* jerk trigger for falls w/o free fall*/
#define RAPID_ROTATION_MDPS     150000  /* 150 dps; sitting/bending < ~100    */
#define STATIONARY_ROTATION_MDPS 20000  /* 20 dps                             */
#define STATIONARY_ACCEL_MIN_MG    850
#define STATIONARY_ACCEL_MAX_MG   1150

/* Added checks */
#define POSTURE_CHANGE_DEG          45  /* upright -> lying                   */
#define INACTIVITY_RATIO_PCT        70  /* share of window spent stationary   */

/* Post-alarm behaviour */
#define RECOVERY_TILT_DEG           30  /* back near the pre-fall posture     */
#define RECOVERY_HOLD_MS          3000
#define LONG_LIE_MS              30000  /* escalate if still down after 30 s  */
#define REARM_DELAY_MS            2000  /* ignore triggers after clearing     */

/* One second of rotation history (used when a fall starts with the impact). */
#define GYRO_HISTORY_LEN   (IMPACT_WINDOW_MS / SAMPLE_PERIOD_MS)

/*------------------------------ Types ---------------------------------------*/
typedef enum {
	FALL_PHASE_MONITORING,
	FALL_PHASE_AWAIT_IMPACT,
	FALL_PHASE_POST_IMPACT,
	FALL_PHASE_ALARM,
	FALL_PHASE_LONG_LIE,
	FALL_PHASE_SOS
} FallPhase;

typedef enum {
	FALL_EVENT_NONE,
	FALL_EVENT_READY,            /* warm-up finished, monitoring started      */
	FALL_EVENT_FREE_FALL,
	FALL_EVENT_IMPACT,
	FALL_EVENT_FALL_CONFIRMED,
	FALL_EVENT_NEAR_FALL,        /* trigger rejected, see last_reason         */
	FALL_EVENT_RECOVERED,
	FALL_EVENT_ACKNOWLEDGED,
	FALL_EVENT_LONG_LIE,
	FALL_EVENT_SOS
} FallEvent;

typedef struct {
	FallPhase phase;
	uint32_t startup_timer_ms;
	uint32_t phase_start_ms;
	uint32_t inactivity_start_ms;
	uint32_t impact_ms;
	uint32_t alarm_ms;
	uint32_t recovery_start_ms;
	uint32_t rearm_until_ms;

	bool is_inactive;
	bool is_free_fall;
	bool is_rapid_rotation;
	bool is_stationary;
	bool is_recovering;
	bool free_fall_seen;

	int prev_accel[3];
	bool has_prev_accel;

	int pre_fall_accel[3];       /* last stationary posture, the reference    */
	bool has_pre_fall_accel;

	/* Per-sample features, also used for the UART report */
	int accel_mag_mg;
	int gyro_mag_mdps;
	int tilt_deg;                /* vs pre_fall_accel                         */

	/* Rotation over the last GYRO_HISTORY_LEN samples, so that rotation
	 * happening just before the trigger still counts as evidence. */
	int gyro_history[GYRO_HISTORY_LEN];
	uint32_t gyro_history_idx;

	/* Evidence collected during a suspected fall */
	uint32_t free_fall_samples;
	int peak_accel_mg;
	int peak_gyro_mdps;
	int32_t post_sum[3];
	uint32_t post_samples;
	uint32_t inactive_samples;

	/* Result of the last evaluation, for the UART log */
	int last_rotation_mdps;
	int last_posture_deg;
	int last_inactive_pct;
	char last_reason[64];
} FallDetector;

/*------------------------------- API ----------------------------------------*/
void       FallDetector_Init(FallDetector *detector, uint32_t now_ms);

/* Call once per sample with the EWMA-filtered accelerometer (mg) and
 * gyroscope (mdps) readings. Returns the event raised by this sample. */
FallEvent  FallDetector_Update(FallDetector *detector, const int accel_mg[3],
                               const int gyro_mdps[3], uint32_t now_ms);

/* User button: short press clears any alarm, long press raises an SOS. */
FallEvent  FallDetector_Acknowledge(FallDetector *detector, uint32_t now_ms);
FallEvent  FallDetector_RequestSOS(FallDetector *detector, uint32_t now_ms);

bool        FallDetector_IsAlarming(const FallDetector *detector);
const char *FallDetector_PhaseName(FallPhase phase);

#endif /* FALL_DETECTOR_H */

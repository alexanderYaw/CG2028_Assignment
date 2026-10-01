/******************************************************************************
 * @file           : main.c
 * @brief          : CG2028 Assignment - ElderCare Wearable Safety Companion
 * @author         : Hou Linxin (starter), CG2028 group
 * (c) CG2028 Teaching Team
 *
 * Data flow, once every SAMPLE_PERIOD_MS:
 *
 *   LSM6DSL accel [mg] + gyro [mdps]
 *        -> ewma_filter (ARM assembly, one recursive state per axis, 6 axes)
 *        -> FallDetector_Update (fall_detector.c): features, phase machine
 *        -> outputs: LED2 blink pattern, buzzer, UART status and event log
 *
 * User button (blue, PC13):
 *   short press during an alarm -> "I am OK" (acknowledge)
 *   hold 2 s when normal        -> manual SOS
 *
 * LED2:
 *   slow blink (1 s)     normal, or a possible fall still being checked
 *   fast blink (150 ms)  fall confirmed
 *   rapid blink (50 ms)  long lie or SOS
 *
 * Grove Buzzer on the Base Shield D3 port. Grove pin 1 (SIG) lands on
 * Arduino D3 = PB0, driven with a square wave from TIM3_CH3 so the pitch
 * (frequency) and volume (duty cycle) can be set:
 *   silent                       normal, or a possible fall still being checked
 *   200 ms beep / 1 s, 700 Hz    fall confirmed
 *   100 ms on/off, 2 kHz         long lie or SOS
 ******************************************************************************/

/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "fall_detector.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

/*--------------------------- Configuration ----------------------------------*/
/* EWMA smoothing. Time constant tau = -T / ln(1 - alpha) with T = 20 ms.
 * Accelerometer 50 % -> tau ~ 29 ms: keeps most of a short impact spike
 *   (an impact lasts only 20-60 ms; 25 % would cut a one-sample spike to a
 *   quarter and soft landings would be missed).
 * Gyroscope 30 % -> tau ~ 56 ms: rotation during a fall lasts 300-800 ms, so
 *   heavier smoothing costs nothing and damps hand tremor and light shaking. */
#define EWMA_ALPHA_ACCEL_PERCENT   50
#define EWMA_ALPHA_GYRO_PERCENT    30

#define NORMAL_LED_DELAY_MS      1000
#define FALL_LED_DELAY_MS         150
#define EMERGENCY_LED_DELAY_MS     50

#define BUZZER_GPIO_Port          ARD_D3_GPIO_Port
#define BUZZER_Pin                ARD_D3_Pin
#define BUZZER_TIMER_TICK_HZ   1000000U  // 1 us steps; tones 16 Hz - 20 kHz
#define BUZZER_VOLUME_PERCENT     30  // 100 = 50 % duty (loudest), 0 = mute
#define BUZZER_STARTUP_CHIRP_MS   100  // confirms the wiring at power-up
#define BUZZER_FALL_FREQ_HZ       700
#define BUZZER_FALL_ON_MS         200
#define BUZZER_FALL_PERIOD_MS    1000
#define BUZZER_EMERGENCY_FREQ_HZ 2000
#define BUZZER_EMERGENCY_ON_MS    100
#define BUZZER_EMERGENCY_PERIOD_MS 200

#define UART_LINE_MAX             256  // longest UART line, incl. timestamp
#define UART_REPORT_PERIOD_MS     200  // status line while monitoring
#define EVAL_REPORT_PERIOD_MS     500  // progress line while checking a fall
#define ALERT_REPEAT_MS          1000  // reminder line during long lie / SOS
#define BUTTON_DEBOUNCE_MS         30
#define BUTTON_LONG_PRESS_MS     2000

#define MG_TO_MPS2          (9.80665f / 1000.0f)

typedef enum {
	BUTTON_EVENT_NONE,
	BUTTON_EVENT_SHORT,
	BUTTON_EVENT_LONG
} ButtonEvent;

/*--------------------------- Prototypes -------------------------------------*/
/* Implemented in mov_avg.s */
extern int ewma_filter(int new_data, int old_output, int alpha_percent);

static void SystemClock_Config(void);
static void UART1_Init(void);
static void Accelerometer_SetRange8g(void);
static void Buzzer_Init(void);

static uint32_t Wait_For_Next_Sample(uint32_t *next_sample_ms);
static bool Sensors_ReadFiltered(int accel_mg[3], int gyro_mdps[3]);
static int ewma_filter_C(int new_data, int old_output, int alpha_percent);

static void LED_Update(FallPhase phase, uint32_t now_ms);
static void Buzzer_SetTone(uint32_t freq_hz);
static void Buzzer_Update(FallPhase phase, uint32_t now_ms);
static ButtonEvent Button_Poll(uint32_t now_ms);

static void Report_Event(FallEvent event, const FallDetector *detector, uint32_t now_ms);
static void Report_Periodic(const FallDetector *detector, const int accel_mg[3],
                            const int gyro_mdps[3], bool event_reported,
                            bool *asm_mismatch, uint32_t now_ms);
static void Report_Status(const FallDetector *detector, const int accel_mg[3],
                          const int gyro_mdps[3], uint32_t now_ms);
static void Report_Evaluation(const FallDetector *detector, uint32_t now_ms);
static void Report_Recovery(const FallDetector *detector, uint32_t now_ms);
static void UART_Log(uint32_t now_ms, const char *format, ...)
    __attribute__((format(printf, 2, 3)));
static void UART_Send(const char *text);

/*--------------------------- Peripheral handles -----------------------------*/
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim3;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    UART1_Init();

    BSP_LED_Init(LED2);
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
    if ((BSP_ACCELERO_Init() != ACCELERO_OK) || (BSP_GYRO_Init() != GYRO_OK))
    {
        UART_Send("\r\nERROR: LSM6DSL motion sensor did not respond.\r\n");
        Error_Handler();
    }
    Accelerometer_SetRange8g();
    BSP_LED_Off(LED2);
    Buzzer_Init();

    FallDetector detector;
    FallDetector_Init(&detector, HAL_GetTick());

    UART_Send("\r\n=== ElderCare Wearable Safety Companion ===\r\n"
              "Hold the board still and upright to set the reference posture.\r\n"
              "Button: short press = I am OK, hold 2 s = SOS.\r\n");

    uint32_t next_sample_ms = HAL_GetTick();
    bool asm_mismatch = false;

    while (1)
    {
        uint32_t now_ms = Wait_For_Next_Sample(&next_sample_ms);

        /* Detection only ever sees the assembly-filtered data. */
        int accel_mg[3];
        int gyro_mdps[3];
        if (Sensors_ReadFiltered(accel_mg, gyro_mdps))
        {
            asm_mismatch = true;
        }

        FallEvent event = FallDetector_Update(&detector, accel_mg, gyro_mdps, now_ms);
        Report_Event(event, &detector, now_ms);

        ButtonEvent button = Button_Poll(now_ms);
        if (button == BUTTON_EVENT_SHORT)
        {
            Report_Event(FallDetector_Acknowledge(&detector, now_ms), &detector, now_ms);
        }
        else if (button == BUTTON_EVENT_LONG)
        {
            Report_Event(FallDetector_RequestSOS(&detector, now_ms), &detector, now_ms);
        }

        LED_Update(detector.phase, now_ms);
        Buzzer_Update(detector.phase, now_ms);
        Report_Periodic(&detector, accel_mg, gyro_mdps,
                        event != FALL_EVENT_NONE, &asm_mismatch, now_ms);
    }
}

/*--------------------------- Sampling ---------------------------------------*/

/* Fixed-rate scheduling: waits for the next SAMPLE_PERIOD_MS slot and returns
 * the current time. If the loop has fallen behind (e.g. a long UART print),
 * the schedule restarts from now instead of running a burst of late samples. */
static uint32_t Wait_For_Next_Sample(uint32_t *next_sample_ms)
{
    while ((int32_t)(HAL_GetTick() - *next_sample_ms) < 0)
    {
    }
    *next_sample_ms += SAMPLE_PERIOD_MS;
    if ((int32_t)(HAL_GetTick() - *next_sample_ms) > SAMPLE_PERIOD_MS)
    {
        *next_sample_ms = HAL_GetTick();
    }
    return HAL_GetTick();
}

/* Reads both sensors and passes every axis through the assembly EWMA filter.
 * The C reference filter runs alongside on its own state purely to verify the
 * assembly; returns true if the two disagree on any axis. */
static bool Sensors_ReadFiltered(int accel_mg[3], int gyro_mdps[3])
{
    /* Previous EWMA outputs: one independent recursive state per axis. */
    static int accel_ewma[3];
    static int gyro_ewma[3];
    static int accel_ewma_c[3];
    static int gyro_ewma_c[3];

    int16_t accel_raw[3] = {0, 0, 0};
    float gyro_raw[3] = {0.0f, 0.0f, 0.0f};
    BSP_ACCELERO_AccGetXYZ(accel_raw);  // mg
    BSP_GYRO_GetXYZ(gyro_raw);  // mdps

    bool mismatch = false;
    for (int axis = 0; axis < 3; axis++)
    {
        int accel_in = accel_raw[axis];
        int gyro_in = (int)gyro_raw[axis];

        accel_ewma[axis] = ewma_filter(accel_in, accel_ewma[axis], EWMA_ALPHA_ACCEL_PERCENT);
        gyro_ewma[axis] = ewma_filter(gyro_in, gyro_ewma[axis], EWMA_ALPHA_GYRO_PERCENT);

        accel_ewma_c[axis] = ewma_filter_C(accel_in, accel_ewma_c[axis], EWMA_ALPHA_ACCEL_PERCENT);
        gyro_ewma_c[axis] = ewma_filter_C(gyro_in, gyro_ewma_c[axis], EWMA_ALPHA_GYRO_PERCENT);

        if ((accel_ewma[axis] != accel_ewma_c[axis]) ||
            (gyro_ewma[axis] != gyro_ewma_c[axis]))
        {
            mismatch = true;
        }

        accel_mg[axis] = accel_ewma[axis];
        gyro_mdps[axis] = gyro_ewma[axis];
    }
    return mismatch;
}

/* Reference implementation for verification only. The assembly routine must
 * be used in the actual sensor-processing and detection pipeline. */
static int ewma_filter_C(int new_data, int old_output, int alpha_percent)
{
    int numerator = alpha_percent * new_data
                  + (100 - alpha_percent) * old_output;
    return numerator / 100;
}

/* The BSP configures the accelerometer for +/-2 g, which clips fall impacts
 * (2-6 g) at 2 g. Switch to +/-8 g; the BSP read function re-reads this
 * register every sample and applies the matching sensitivity, so the readings
 * stay in mg. */
static void Accelerometer_SetRange8g(void)
{
    const uint8_t full_scale_mask = 0x0C;  // FS_XL bits of CTRL1_XL
    uint8_t ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
                                  LSM6DSL_ACC_GYRO_CTRL1_XL);
    ctrl = (uint8_t)((ctrl & ~full_scale_mask) | LSM6DSL_ACC_FULLSCALE_8G);
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
                    LSM6DSL_ACC_GYRO_CTRL1_XL, ctrl);
}

/*--------------------------- LED, buzzer and button -------------------------*/

/* Non-blocking LED blinking: the rate follows the phase and never delays
 * sampling. A change of pattern takes effect immediately. */
static void LED_Update(FallPhase phase, uint32_t now_ms)
{
    static uint32_t last_toggle_ms = 0;
    static uint32_t current_period_ms = NORMAL_LED_DELAY_MS;

    uint32_t period_ms;
    switch (phase)
    {
    case FALL_PHASE_ALARM:
        period_ms = FALL_LED_DELAY_MS;
        break;
    case FALL_PHASE_LONG_LIE:
    case FALL_PHASE_SOS:
        period_ms = EMERGENCY_LED_DELAY_MS;
        break;
    default:
        period_ms = NORMAL_LED_DELAY_MS;
        break;
    }

    if (period_ms != current_period_ms)
    {
        current_period_ms = period_ms;
        last_toggle_ms = now_ms - period_ms;  // toggle right away
    }
    if ((now_ms - last_toggle_ms) >= current_period_ms)
    {
        last_toggle_ms = now_ms;
        BSP_LED_Toggle(LED2);
    }
}

/* PB0 is TIM3_CH3 (AF2). TIM3 counts 1 us ticks; the auto-reload sets the
 * tone period and the compare value sets the duty cycle. A short chirp at
 * start-up confirms the wiring before monitoring begins. */
static void Buzzer_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    GPIO_InitStruct.Pin = BUZZER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

    /* APB1 is not divided, so the TIM3 clock equals PCLK1 (80 MHz). */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = HAL_RCC_GetPCLK1Freq() / BUZZER_TIMER_TICK_HZ - 1U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;  // output held low: silent
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if ((HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_3) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK))
    {
        Error_Handler();
    }

    Buzzer_SetTone(BUZZER_FALL_FREQ_HZ);
    HAL_Delay(BUZZER_STARTUP_CHIRP_MS);
    Buzzer_SetTone(0);
}

/* Plays a square wave at freq_hz (16 Hz - 20 kHz), or silences the buzzer
 * when freq_hz is 0. Registers are only touched when the tone changes. */
static void Buzzer_SetTone(uint32_t freq_hz)
{
    static uint32_t current_hz = 0;

    if (freq_hz == current_hz)
    {
        return;
    }
    current_hz = freq_hz;

    if (freq_hz == 0U)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    }
    else
    {
        uint32_t period_ticks = BUZZER_TIMER_TICK_HZ / freq_hz;
        __HAL_TIM_SET_AUTORELOAD(&htim3, period_ticks - 1U);
        /* A 50 % duty cycle drives the buzzer hardest; a narrower pulse
         * carries less energy at the tone frequency, so it sounds quieter. */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
                              period_ticks * BUZZER_VOLUME_PERCENT / 200U);
    }
    /* Load the preloaded values now rather than at the end of the old period. */
    HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
}

/* Non-blocking beep pattern that follows the phase, like LED_Update. A new
 * pattern starts with a beep straight away. */
static void Buzzer_Update(FallPhase phase, uint32_t now_ms)
{
    static FallPhase current_phase = FALL_PHASE_MONITORING;
    static uint32_t pattern_start_ms = 0;

    uint32_t freq_hz;
    uint32_t on_ms;
    uint32_t period_ms;
    switch (phase)
    {
    case FALL_PHASE_ALARM:
        freq_hz = BUZZER_FALL_FREQ_HZ;
        on_ms = BUZZER_FALL_ON_MS;
        period_ms = BUZZER_FALL_PERIOD_MS;
        break;
    case FALL_PHASE_LONG_LIE:
    case FALL_PHASE_SOS:
        freq_hz = BUZZER_EMERGENCY_FREQ_HZ;
        on_ms = BUZZER_EMERGENCY_ON_MS;
        period_ms = BUZZER_EMERGENCY_PERIOD_MS;
        break;
    default:
        freq_hz = 0;
        on_ms = 0;
        period_ms = 1;
        break;
    }

    if (phase != current_phase)
    {
        current_phase = phase;
        pattern_start_ms = now_ms;
    }

    bool on = ((now_ms - pattern_start_ms) % period_ms) < on_ms;
    Buzzer_SetTone(on ? freq_hz : 0U);
}

/* Debounced user button (active low). SHORT is reported on release, LONG once
 * the button has been held for BUTTON_LONG_PRESS_MS. */
static ButtonEvent Button_Poll(uint32_t now_ms)
{
    static bool stable_pressed = false;
    static bool last_raw = false;
    static bool long_reported = false;
    static uint32_t last_change_ms = 0;
    static uint32_t press_start_ms = 0;

    bool raw = (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET);
    ButtonEvent event = BUTTON_EVENT_NONE;

    if (raw != last_raw)
    {
        last_raw = raw;
        last_change_ms = now_ms;
    }

    if ((raw != stable_pressed) && ((now_ms - last_change_ms) >= BUTTON_DEBOUNCE_MS))
    {
        stable_pressed = raw;
        if (stable_pressed)
        {
            press_start_ms = now_ms;
            long_reported = false;
        }
        else if (!long_reported)
        {
            event = BUTTON_EVENT_SHORT;
        }
    }

    if (stable_pressed && !long_reported &&
        ((now_ms - press_start_ms) >= BUTTON_LONG_PRESS_MS))
    {
        long_reported = true;
        event = BUTTON_EVENT_LONG;
    }
    return event;
}

/*--------------------------- UART reporting ---------------------------------*/

/* One line per detector event. */
static void Report_Event(FallEvent event, const FallDetector *detector, uint32_t now_ms)
{
    switch (event)
    {
    case FALL_EVENT_READY:
        UART_Log(now_ms, "Reference posture set to (%d, %d, %d) mg. Monitoring.\r\n",
                 detector->pre_fall_accel[0], detector->pre_fall_accel[1],
                 detector->pre_fall_accel[2]);
        break;
    case FALL_EVENT_FREE_FALL:
        UART_Log(now_ms, ">> Possible fall (|a| = %d mg, |w| = %d dps) - waiting for impact\r\n",
                 detector->accel_mag_mg, detector->gyro_mag_mdps / 1000);
        break;
    case FALL_EVENT_IMPACT:
        UART_Log(now_ms, ">> Impact %.2f g - checking rotation, posture, inactivity\r\n",
                 detector->accel_mag_mg / 1000.0f);
        break;
    case FALL_EVENT_FALL_CONFIRMED:
        UART_Log(now_ms, "*** FALL CONFIRMED: impact %.2f g, rotation %d dps, "
                 "posture change %d deg, inactive %d%% ***\r\n",
                 detector->peak_accel_mg / 1000.0f,
                 detector->last_rotation_mdps / 1000,
                 detector->last_posture_deg, detector->last_inactive_pct);
        break;
    case FALL_EVENT_NEAR_FALL:
        UART_Log(now_ms, "-- Near-fall rejected (%s): rotation %d dps, "
                 "posture change %d deg, inactive %d%%\r\n",
                 detector->last_reason, detector->last_rotation_mdps / 1000,
                 detector->last_posture_deg, detector->last_inactive_pct);
        break;
    case FALL_EVENT_RECOVERED:
        UART_Log(now_ms, "-- Recovered: upright and steady for %lu s. Monitoring.\r\n",
                 (unsigned long)(RECOVERY_HOLD_MS / 1000U));
        break;
    case FALL_EVENT_ACKNOWLEDGED:
        UART_Log(now_ms, "-- Alarm acknowledged by user. Monitoring.\r\n");
        break;
    case FALL_EVENT_LONG_LIE:
        UART_Log(now_ms, "!!! LONG LIE: no recovery %lu s after the fall - summon help !!!\r\n",
                 (unsigned long)(LONG_LIE_MS / 1000U));
        break;
    case FALL_EVENT_SOS:
        UART_Log(now_ms, "!!! SOS requested by user !!!\r\n");
        break;
    case FALL_EVENT_NONE:
    default:
        break;
    }
}

/* Periodic line, chosen by phase: recovery progress during an alarm, a
 * reminder during long lie / SOS, evaluation progress while a possible fall
 * is checked, and the sensor status otherwise. After an event line
 * (event_reported) the next periodic line waits a full period. */
static void Report_Periodic(const FallDetector *detector, const int accel_mg[3],
                            const int gyro_mdps[3], bool event_reported,
                            bool *asm_mismatch, uint32_t now_ms)
{
    static uint32_t last_report_ms = 0;
    static uint32_t last_alert_ms = 0;

    if (event_reported)
    {
        last_report_ms = now_ms;
    }

    switch (detector->phase)
    {
    case FALL_PHASE_ALARM:
        if ((now_ms - last_alert_ms) >= EVAL_REPORT_PERIOD_MS)
        {
            last_alert_ms = now_ms;
            Report_Recovery(detector, now_ms);
        }
        break;

    case FALL_PHASE_LONG_LIE:
    case FALL_PHASE_SOS:
        if ((now_ms - last_alert_ms) >= ALERT_REPEAT_MS)
        {
            last_alert_ms = now_ms;
            UART_Log(now_ms, "!!! %s - %lu s - press USER button if OK\r\n",
                     FallDetector_PhaseName(detector->phase),
                     (unsigned long)((now_ms - detector->phase_start_ms) / 1000U));
        }
        break;

    case FALL_PHASE_AWAIT_IMPACT:
    case FALL_PHASE_POST_IMPACT:
        if ((now_ms - last_report_ms) >= EVAL_REPORT_PERIOD_MS)
        {
            last_report_ms = now_ms;
            Report_Evaluation(detector, now_ms);
        }
        break;

    case FALL_PHASE_MONITORING:
    default:
        if ((now_ms - last_report_ms) >= UART_REPORT_PERIOD_MS)
        {
            last_report_ms = now_ms;
            Report_Status(detector, accel_mg, gyro_mdps, now_ms);
            if (*asm_mismatch)
            {
                UART_Send("WARNING: Assembly and C EWMA outputs do not match.\r\n");
                *asm_mismatch = false;
            }
        }
        break;
    }
}

static void Report_Status(const FallDetector *detector, const int accel_mg[3],
                          const int gyro_mdps[3], uint32_t now_ms)
{
    UART_Log(now_ms,
             "%-13s %6.1f s |a|=%.2f g |w|=%4d dps tilt=%3d deg | "
             "A[m/s^2] %6.2f %6.2f %6.2f | G[dps] %7.1f %7.1f %7.1f\r\n",
             FallDetector_PhaseName(detector->phase),
             (now_ms - detector->phase_start_ms) / 1000.0f,
             detector->accel_mag_mg / 1000.0f,
             detector->gyro_mag_mdps / 1000,
             detector->tilt_deg,
             accel_mg[0] * MG_TO_MPS2, accel_mg[1] * MG_TO_MPS2, accel_mg[2] * MG_TO_MPS2,
             gyro_mdps[0] / 1000.0f, gyro_mdps[1] / 1000.0f, gyro_mdps[2] / 1000.0f);
}

static void Report_Evaluation(const FallDetector *detector, uint32_t now_ms)
{
    uint32_t in_phase_ms = now_ms - detector->phase_start_ms;
    const char *deep = (detector->min_accel_mg < DEEP_FREE_FALL_MG) ? " (deep)" : "";

    if (detector->phase == FALL_PHASE_AWAIT_IMPACT)
    {
        UART_Log(now_ms,
                 "%-9s %4.1f/%.1f s | waiting for impact | |a|=%.2f g | "
                 "lowest |a| %d mg%s | rotation %d dps\r\n",
                 FallDetector_PhaseName(detector->phase),
                 in_phase_ms / 1000.0f, IMPACT_WINDOW_MS / 1000.0f,
                 detector->accel_mag_mg / 1000.0f,
                 detector->min_accel_mg, deep,
                 detector->peak_gyro_mdps / 1000);
        return;
    }

    char still[32];
    if (in_phase_ms < POST_IMPACT_SETTLE_MS)
    {
        snprintf(still, sizeof(still), "settling");
    }
    else
    {
        uint32_t still_ms = (detector->post_samples > 0U)
                          ? (now_ms - detector->inactivity_start_ms) : 0U;
        snprintf(still, sizeof(still), "still %.2f/%.2f s",
                 still_ms / 1000.0f, INACTIVITY_REQUIRED_MS / 1000.0f);
    }
    UART_Log(now_ms,
             "%-9s %4.1f/%.0f s | %s | rotation %d dps | "
             "lowest |a| %d mg%s | tilt %d deg\r\n",
             FallDetector_PhaseName(detector->phase),
             in_phase_ms / 1000.0f, POST_IMPACT_WINDOW_MS / 1000.0f,
             still,
             detector->peak_gyro_mdps / 1000,
             detector->min_accel_mg, deep,
             detector->tilt_deg);
}

/* During an alarm: shows which self-recovery step the wearer is on. */
static void Report_Recovery(const FallDetector *detector, uint32_t now_ms)
{
    char step[48];
    if (!detector->moved_since_alarm)
    {
        snprintf(step, sizeof(step), "first tilt >= %d deg away", RECOVERY_LEAVE_DEG);
    }
    else if (detector->tilt_deg >= RECOVERY_TILT_DEG)
    {
        snprintf(step, sizeof(step), "return to < %d deg", RECOVERY_TILT_DEG);
    }
    else if (!detector->is_recovering)
    {
        snprintf(step, sizeof(step), "hold still (|w|=%d dps)",
                 detector->gyro_mag_mdps / 1000);
    }
    else
    {
        snprintf(step, sizeof(step), "still %.1f/%.1f s",
                 (now_ms - detector->recovery_start_ms) / 1000.0f,
                 RECOVERY_HOLD_MS / 1000.0f);
    }

    UART_Log(now_ms,
             "!!! %s %4.1f/%.0f s | press USER button if OK | "
             "recovery: tilt %d deg, %s\r\n",
             FallDetector_PhaseName(detector->phase),
             (now_ms - detector->phase_start_ms) / 1000.0f, LONG_LIE_MS / 1000.0f,
             detector->tilt_deg, step);
}

/* printf-style UART line prefixed with a "[seconds.hundredths s]" timestamp.
 * Lines longer than UART_LINE_MAX are truncated. */
static void UART_Log(uint32_t now_ms, const char *format, ...)
{
    char buffer[UART_LINE_MAX];
    int stamp_len = snprintf(buffer, sizeof(buffer), "[%6lu.%02lu s] ",
                             (unsigned long)(now_ms / 1000U),
                             (unsigned long)((now_ms % 1000U) / 10U));

    va_list args;
    va_start(args, format);
    vsnprintf(buffer + stamp_len, sizeof(buffer) - (size_t)stamp_len, format, args);
    va_end(args);

    UART_Send(buffer);
}

/* Blocking transmit: at 115200 baud a 150-character line takes ~13 ms. */
static void UART_Send(const char *text)
{
    HAL_UART_Transmit(&huart1, (const uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
}

/*--------------------------- Hardware setup ---------------------------------*/

static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = ST_LINK_UART1_TX_Pin | ST_LINK_UART1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ST_LINK_UART1_TX_GPIO_Port, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* 80 MHz SYSCLK from the 4 MHz MSI through the PLL; AHB and both APB buses
 * run undivided. */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    osc.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    osc.MSIState = RCC_MSI_ON;
    osc.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    osc.MSIClockRange = RCC_MSIRANGE_6;  // 4 MHz
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    osc.PLL.PLLM = 1;
    osc.PLL.PLLN = 40;  // VCO 160 MHz
    osc.PLL.PLLP = 2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc.PLL.PLLR = RCC_PLLR_DIV2;  // SYSCLK 80 MHz
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Unrecoverable hardware set-up failure: stop here with interrupts off so the
 * fault is easy to find in the debugger. Declared in main.h. */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

/* Do not modify these lines. They suppress UART-related warnings. */
int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}
int _read(int file, char *ptr, int len) { (void)file; (void)ptr; (void)len; return 0; }
int _fstat(int file, struct stat *st) { (void)file; (void)st; return 0; }
int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return 0; }
int _isatty(int file) { (void)file; return 1; }
int _close(int file) { (void)file; return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }

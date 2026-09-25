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
 *        -> outputs: LED2 blink pattern, UART status and event log
 *
 * User button (blue, PC13):
 *   short press during an alarm -> "I am OK" (acknowledge)
 *   hold 2 s when normal        -> manual SOS
 *
 * LED2:
 *   slow blink (1 s)     normal, or a possible fall still being checked
 *   fast blink (150 ms)  fall confirmed
 *   rapid blink (50 ms)  long lie or SOS
 ******************************************************************************/

/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "fall_detector.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"

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

#define ALERT_REPEAT_MS          1000
#define BUTTON_DEBOUNCE_MS         30
#define BUTTON_LONG_PRESS_MS     2000

#define MG_TO_MPS2          (9.80665f / 1000.0f)

typedef enum {
	BUTTON_EVENT_NONE,
	BUTTON_EVENT_SHORT,
	BUTTON_EVENT_LONG
} ButtonEvent;

static void UART1_Init(void);
static void UART_Send(const char *text);
static void Accelerometer_SetRange8g(void);
static void LED_Update(FallPhase phase, uint32_t now_ms);
static ButtonEvent Button_Poll(uint32_t now_ms);
static void Report_Event(FallEvent event, const FallDetector *detector, uint32_t now_ms);
static void Report_Status(const FallDetector *detector, const int accel_mg[3],
                          const int gyro_mdps[3], uint32_t now_ms);

extern int ewma_filter(int new_data, int old_output, int alpha_percent);
int ewma_filter_C(int new_data, int old_output, int alpha_percent);

UART_HandleTypeDef huart1;

int main(void)
{
    HAL_Init();
    UART1_Init();

    BSP_LED_Init(LED2);
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    Accelerometer_SetRange8g();
    BSP_LED_Off(LED2);

    /* Previous EWMA outputs: one independent recursive state per axis. */
    int accel_ewma_asm[3] = {0, 0, 0};
    int gyro_ewma_asm[3]  = {0, 0, 0};

    /* Reference C states are kept separately for assembly verification. */
    int accel_ewma_c[3] = {0, 0, 0};
    int gyro_ewma_c[3]  = {0, 0, 0};

    FallDetector detector;
    FallDetector_Init(&detector, HAL_GetTick());

    UART_Send("\r\n=== ElderCare Wearable Safety Companion ===\r\n"
              "Hold the board still and upright to set the reference posture.\r\n"
              "Button: short press = I am OK, hold 2 s = SOS.\r\n");

    uint32_t next_sample_ms = HAL_GetTick();
    uint32_t last_report_ms = 0;
    uint32_t last_alert_ms = 0;
    bool asm_mismatch = false;

    while (1)
    {
        /*---------------- Fixed-rate scheduling ----------------*/
        while ((int32_t)(HAL_GetTick() - next_sample_ms) < 0)
        {
        }
        next_sample_ms += SAMPLE_PERIOD_MS;
        if ((int32_t)(HAL_GetTick() - next_sample_ms) > SAMPLE_PERIOD_MS)
        {
            next_sample_ms = HAL_GetTick();   /* fell behind, e.g. long print */
        }
        uint32_t now_ms = HAL_GetTick();

        /*---------------- Sensor acquisition ----------------*/
        int16_t accel_raw_i16[3] = {0, 0, 0};
        float gyro_raw_float[3] = {0.0f, 0.0f, 0.0f};
        int gyro_raw_int[3] = {0, 0, 0};

        BSP_ACCELERO_AccGetXYZ(accel_raw_i16);   /* mg   */
        BSP_GYRO_GetXYZ(gyro_raw_float);         /* mdps */

        /*---------------- EWMA filtering (assembly) ----------------*/
        for (int axis = 0; axis < 3; axis++)
        {
            gyro_raw_int[axis] = (int)gyro_raw_float[axis];

            accel_ewma_asm[axis] = ewma_filter(
                (int)accel_raw_i16[axis],
                accel_ewma_asm[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_asm[axis] = ewma_filter(
                gyro_raw_int[axis],
                gyro_ewma_asm[axis],
                EWMA_ALPHA_GYRO_PERCENT);

            accel_ewma_c[axis] = ewma_filter_C(
                (int)accel_raw_i16[axis],
                accel_ewma_c[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_c[axis] = ewma_filter_C(
                gyro_raw_int[axis],
                gyro_ewma_c[axis],
                EWMA_ALPHA_GYRO_PERCENT);

            if ((accel_ewma_asm[axis] != accel_ewma_c[axis]) ||
                (gyro_ewma_asm[axis] != gyro_ewma_c[axis]))
            {
                asm_mismatch = true;
            }
        }

        /*---------------- Fall detection (filtered data only) ----------------*/
        FallEvent event = FallDetector_Update(&detector, accel_ewma_asm,
                                              gyro_ewma_asm, now_ms);
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

        /*---------------- Outputs ----------------*/
        LED_Update(detector.phase, now_ms);

        /* Nothing is printed while a possible fall is being evaluated, so the
         * blocking UART cannot delay a sample in the critical window. */
        bool evaluating = (detector.phase == FALL_PHASE_AWAIT_IMPACT) ||
                          (detector.phase == FALL_PHASE_POST_IMPACT);

        if (FallDetector_IsAlarming(&detector))
        {
            if ((now_ms - last_alert_ms) >= ALERT_REPEAT_MS)
            {
                last_alert_ms = now_ms;
                char buffer[96];
                snprintf(buffer, sizeof(buffer),
                         "!!! %s - %lu s - press USER button if OK\r\n",
                         FallDetector_PhaseName(detector.phase),
                         (unsigned long)((now_ms - detector.phase_start_ms) / 1000U));
                UART_Send(buffer);
            }
        }
        else if (!evaluating && ((now_ms - last_report_ms) >= UART_REPORT_PERIOD_MS))
        {
            last_report_ms = now_ms;
            Report_Status(&detector, accel_ewma_asm, gyro_ewma_asm, now_ms);
            if (asm_mismatch)
            {
                UART_Send("WARNING: Assembly and C EWMA outputs do not match.\r\n");
                asm_mismatch = false;
            }
        }
    }
}

int ewma_filter_C(int new_data, int old_output, int alpha_percent)
{
    /* Reference implementation for verification only. The assembly routine
     * must be used in the actual sensor-processing and detection pipeline. */
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
    const uint8_t full_scale_mask = 0x0C;   /* FS_XL bits of CTRL1_XL */
    uint8_t ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
                                  LSM6DSL_ACC_GYRO_CTRL1_XL);
    ctrl = (uint8_t)((ctrl & ~full_scale_mask) | LSM6DSL_ACC_FULLSCALE_8G);
    SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW,
                    LSM6DSL_ACC_GYRO_CTRL1_XL, ctrl);
}

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
        last_toggle_ms = now_ms - period_ms;   /* toggle right away */
    }
    if ((now_ms - last_toggle_ms) >= current_period_ms)
    {
        last_toggle_ms = now_ms;
        BSP_LED_Toggle(LED2);
    }
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

static void Report_Event(FallEvent event, const FallDetector *detector, uint32_t now_ms)
{
    char buffer[200];
    buffer[0] = '\0';

    switch (event)
    {
    case FALL_EVENT_READY:
        snprintf(buffer, sizeof(buffer),
                 "Reference posture set to (%d, %d, %d) mg. Monitoring.\r\n",
                 detector->pre_fall_accel[0], detector->pre_fall_accel[1],
                 detector->pre_fall_accel[2]);
        break;
    case FALL_EVENT_FREE_FALL:
        snprintf(buffer, sizeof(buffer),
                 ">> Possible fall (|a| = %d mg, |w| = %d dps) - waiting for impact\r\n",
                 detector->accel_mag_mg, detector->gyro_mag_mdps / 1000);
        break;
    case FALL_EVENT_IMPACT:
        snprintf(buffer, sizeof(buffer),
                 ">> Impact %.2f g - checking rotation, posture, inactivity\r\n",
                 detector->accel_mag_mg / 1000.0f);
        break;
    case FALL_EVENT_FALL_CONFIRMED:
        snprintf(buffer, sizeof(buffer),
                 "*** FALL CONFIRMED: impact %.2f g, rotation %d dps, "
                 "posture change %d deg, inactive %d%% ***\r\n",
                 detector->peak_accel_mg / 1000.0f,
                 detector->last_rotation_mdps / 1000,
                 detector->last_posture_deg, detector->last_inactive_pct);
        break;
    case FALL_EVENT_NEAR_FALL:
        snprintf(buffer, sizeof(buffer),
                 "-- Near-fall rejected (%s): rotation %d dps, "
                 "posture change %d deg, inactive %d%%\r\n",
                 detector->last_reason, detector->last_rotation_mdps / 1000,
                 detector->last_posture_deg, detector->last_inactive_pct);
        break;
    case FALL_EVENT_RECOVERED:
        snprintf(buffer, sizeof(buffer),
                 "-- Recovered: upright and steady for %lu s. Monitoring.\r\n",
                 (unsigned long)(RECOVERY_HOLD_MS / 1000U));
        break;
    case FALL_EVENT_ACKNOWLEDGED:
        snprintf(buffer, sizeof(buffer), "-- Alarm acknowledged by user. Monitoring.\r\n");
        break;
    case FALL_EVENT_LONG_LIE:
        snprintf(buffer, sizeof(buffer),
                 "!!! LONG LIE: no recovery %lu s after the fall - summon help !!!\r\n",
                 (unsigned long)(LONG_LIE_MS / 1000U));
        break;
    case FALL_EVENT_SOS:
        snprintf(buffer, sizeof(buffer), "!!! SOS requested by user !!!\r\n");
        break;
    case FALL_EVENT_NONE:
    default:
        break;
    }

    if (buffer[0] != '\0')
    {
        char stamped[220];
        snprintf(stamped, sizeof(stamped), "[%6lu.%02lu s] %s",
                 (unsigned long)(now_ms / 1000U),
                 (unsigned long)((now_ms % 1000U) / 10U), buffer);
        UART_Send(stamped);
    }
}

static void Report_Status(const FallDetector *detector, const int accel_mg[3],
                          const int gyro_mdps[3], uint32_t now_ms)
{
    char buffer[220];
    snprintf(buffer, sizeof(buffer),
             "[%6lu.%02lu s] %-13s |a|=%.2f g |w|=%4d dps tilt=%3d deg | "
             "A[m/s^2] %6.2f %6.2f %6.2f | G[dps] %7.1f %7.1f %7.1f\r\n",
             (unsigned long)(now_ms / 1000U),
             (unsigned long)((now_ms % 1000U) / 10U),
             FallDetector_PhaseName(detector->phase),
             detector->accel_mag_mg / 1000.0f,
             detector->gyro_mag_mdps / 1000,
             detector->tilt_deg,
             accel_mg[0] * MG_TO_MPS2, accel_mg[1] * MG_TO_MPS2, accel_mg[2] * MG_TO_MPS2,
             gyro_mdps[0] / 1000.0f, gyro_mdps[1] / 1000.0f, gyro_mdps[2] / 1000.0f);
    UART_Send(buffer);
}

static void UART_Send(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
        while (1) { }
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

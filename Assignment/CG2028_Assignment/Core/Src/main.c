/******************************************************************************
 * @file           : main.c
 * @brief          : CG2028 Assignment - ElderCare Wearable Safety Companion
 * @author         : Hou Linxin
 * (c) CG2028 Teaching Team
 ******************************************************************************/

/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <stdbool.h>

/*--------------------------- Configuration ----------------------------------*/
#define EWMA_ALPHA_ACCEL_PERCENT   25
#define EWMA_ALPHA_GYRO_PERCENT    25
#define NORMAL_LED_DELAY_MS       1000
#define FALL_LED_DELAY_MS          150

/*--------------------------- Sampling ----------------------------------*/
#define SAMPLE_PERIOD_MS           20  // 50Hz
#define STARTUP_WARMUP_MS          500
#define IMPACT_WINDOW_MS          1000
#define POST_IMPACT_WINDOW_MS     2000
#define INACTIVITY_REQUIRED_MS     750
#define UART_REPORT_PERIOD_MS      200

/*--------------------------- Thresholds ----------------------------------*/
#define FREE_FALL_MG               650
#define IMPACT_MG                 1800
#define SUDDEN_CHANGE_MG           600
#define RAPID_ROTATION_MDPS      150000
#define STATIONARY_ROTATION_MDPS  20000
#define STATIONARY_ACCEL_MIN_MG    850
#define STATIONARY_ACCEL_MAX_MG   1150

/*--------------------------- Falling Phases and Phase detector ----------------------------------*/
typedef enum {
	FALL_PHASE_MONITORING,
	FALL_PHASE_AWAIT_IMPACT,
	FALL_PHASE_POST_IMPACT,
	FALL_PHASE_ALARM
} FallPhase;

typedef struct {
	FallPhase phase;
	uint32_t startup_timer_ms;
	uint32_t phase_start_ms;
	uint32_t inactivity_start_ms;

	bool is_inactive;
	bool is_free_fall;
	bool is_rapid_rotation;

	int prev_accel[3];
	bool has_prev_accel;

	int pre_fall_accel[3];
	bool has_pre_fall_accel;
} FallDetector;

static void UART1_Init(void);
static void UART_Send(const char *text);

extern int ewma_filter(int new_data, int old_output, int alpha_percent);
//int ewma_filter_C(int new_data, int old_output, int alpha_percent);

UART_HandleTypeDef huart1;

static uint64_t square (int64_t input) {
	return (uint64_t)(input * input);
}

static uint64_t square_vector_magnitude(int vector[3]) {
	int64_t x = vector[0];
	int64_t y = vector[1];
	int64_t z = vector[2];

	return (uint64_t)((x * x) + (y * y) + (z * z));
}

static bool is_vector_change_exceeded(const int curr_vector[3], const int prev_vector[3], uint32_t threshold) {
	uint64_t delta = 0;
	uint64_t threshold_sqrd;

	for (int i = 0; i < 3; i++) {
		delta += square((int64_t)curr_vector[i] - prev_vector[i]);
	}

	threshold_sqrd = square(threshold);

	if (delta <= threshold_sqrd) {
		return 0;
	}

	return 1;
}

int main(void)
{
    HAL_Init();
    UART1_Init();

    BSP_LED_Init(LED2);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_LED_Off(LED2);

    /* Previous EWMA outputs. The first test/application sample starts from 0. */
    int accel_ewma_asm[3] = {0, 0, 0};
    int gyro_ewma_asm[3]  = {0, 0, 0};

    /* Reference C states are kept separately for assembly verification. */
    int accel_ewma_c[3] = {0, 0, 0};
    int gyro_ewma_c[3]  = {0, 0, 0};

    unsigned long sample_number = 0;

    /* Initialise the fall detector */
    FallDetector detector = {0};
    detector.phase = FALL_PHASE_MONITORING;
    detector.startup_timer_ms = HAL_GetTick();

    while (1)
    {
        int16_t accel_raw_i16[3] = {0, 0, 0};
        float gyro_raw_float[3] = {0.0f, 0.0f, 0.0f};
        int gyro_raw_int[3] = {0, 0, 0};

        BSP_ACCELERO_AccGetXYZ(accel_raw_i16);
        BSP_GYRO_GetXYZ(gyro_raw_float);

        /* The supplied BSP reports gyroscope readings as floating-point raw
         * values. Convert them to signed integers before passing them to the
         * integer assembly routine. */
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
        }

        uint64_t accel_mag_sqrd = square_vector_magnitude(accel_ewma_asm);
        uint64_t gyro_mag_sqrd = square_vector_magnitude(gyro_ewma_asm);

        bool possible_free_fall = (accel_mag_sqrd < square(FREE_FALL_MG));
        bool impact_detected = (accel_mag_sqrd > square(IMPACT_MG));
        bool rapid_rotation = (gyro_mag_sqrd > square(RAPID_ROTATION_MDPS));
        bool is_stationary = (accel_mag_sqrd > square(STATIONARY_ACCEL_MIN_MG)) &&
        			(accel_mag_sqrd < square(STATIONARY_ACCEL_MAX_MG));

        /* Accelerometer filtered readings are in meters per second squared. */
        float accel_mps2[3] = {
            accel_ewma_asm[0] * (9.80665f / 1000.0f),
            accel_ewma_asm[1] * (9.80665f / 1000.0f),
            accel_ewma_asm[2] * (9.80665f / 1000.0f)
        };

        /* Gyroscope filtered readings are in degrees per second. */
        float gyro_dps[3] = {
            gyro_ewma_asm[0] / 1000.0f,
            gyro_ewma_asm[1] / 1000.0f,
            gyro_ewma_asm[2] / 1000.0f
        };

        char buffer[320];
        snprintf(buffer, sizeof(buffer),
                 "Sample %lu\r\n"
                 "Accel EWMA ASM [m/s^2]: X=%8.3f Y=%8.3f Z=%8.3f\r\n"
                 "Gyro  EWMA ASM [dps]  : X=%8.3f Y=%8.3f Z=%8.3f\r\n",
                 sample_number,
                 accel_mps2[0], accel_mps2[1], accel_mps2[2],
                 gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        UART_Send(buffer);

        /* Optional debugging check. This confirms that the assembly routine
         * matches the reference C routine for the current samples. */
        if ((accel_ewma_asm[0] != accel_ewma_c[0]) ||
            (accel_ewma_asm[1] != accel_ewma_c[1]) ||
            (accel_ewma_asm[2] != accel_ewma_c[2]) ||
            (gyro_ewma_asm[0] != gyro_ewma_c[0]) ||
            (gyro_ewma_asm[1] != gyro_ewma_c[1]) ||
            (gyro_ewma_asm[2] != gyro_ewma_c[2]))
        {
            UART_Send("WARNING: Assembly and C EWMA outputs do not match.\r\n");
        }

        /**************** Elderly wearable state logic starts here************************
         * Compulsory requirements:
         * 1. Use filtered accelerometer AND gyroscope readings.
         * 2. Distinguish normal activity, near-fall movements, and a real fall.
         * 3. Use a slow LED blink for normal operation and a fast blink after
         *    a fall is detected.
         *********************************************************************/

        int fall_detected = 0;  /* TODO: replace with your fall-detection logic */

        BSP_LED_Toggle(LED2);
        HAL_Delay(fall_detected ? FALL_LED_DELAY_MS : NORMAL_LED_DELAY_MS);

        sample_number++;
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

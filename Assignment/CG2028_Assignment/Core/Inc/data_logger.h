/******************************************************************************
 * @file           : data_logger.h
 * @brief          : CG2028 Assignment - CSV data logging for threshold tuning
 *
 * With DATA_LOG_MODE set to 1, main.c streams one CSV line per sample over
 * UART instead of the human-readable reports. The raw columns can be replayed
 * on a PC through the EWMA and fall_detector.c with any alpha or threshold
 * (Part2_Simulation/replay.c).
 *
 * Like fall_detector.c, this module never touches the HAL: it only formats
 * text, and main.c sends it.
 ******************************************************************************/

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "fall_detector.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 0: normal demo build - LEAVE AT 0 for the demo and the submission.
 * 1: CSV stream for recording trials with Part2_Simulation/capture.py. */
#define DATA_LOG_MODE              0

typedef struct {
	uint32_t  t_ms;
	int       accel_raw_mg[3];
	int       gyro_raw_mdps[3];
	int       accel_filt_mg[3];      /* assembly EWMA output */
	int       gyro_filt_mdps[3];     /* assembly EWMA output */
	FallPhase phase;
	FallEvent event;
	bool      asm_matches_c;         /* all 6 axes agree with ewma_filter_C */
} DataLogSample;

/* '#' comment lines describing the configuration, then the CSV column names.
 * Returns the number of characters snprintf wanted to write. */
int DataLogger_FormatHeader(char *buffer, size_t size, int alpha_accel_percent,
                            int alpha_gyro_percent, int sample_period_ms);

/* One CSV line, terminated by \r\n. */
int DataLogger_FormatSample(char *buffer, size_t size, const DataLogSample *sample);

#endif /* DATA_LOGGER_H */

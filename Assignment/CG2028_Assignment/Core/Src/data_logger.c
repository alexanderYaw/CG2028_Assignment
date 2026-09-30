/******************************************************************************
 * @file           : data_logger.c
 * @brief          : CG2028 Assignment - CSV data logging for threshold tuning
 *
 * See data_logger.h. Integers only, so no printf float support is needed and
 * a line takes well under the 20 ms sample period to send.
 ******************************************************************************/

#include "data_logger.h"

#include <stdio.h>

int DataLogger_FormatHeader(char *buffer, size_t size, int alpha_accel_percent,
                            int alpha_gyro_percent, int sample_period_ms)
{
	return snprintf(buffer, size,
	                "# ElderCare data log\r\n"
	                "# alpha_accel=%d alpha_gyro=%d sample_period_ms=%d accel_range=8g\r\n"
	                "# units: accel mg, gyro mdps; f* columns = assembly EWMA output\r\n"
	                "t_ms,ax,ay,az,gx,gy,gz,fax,fay,faz,fgx,fgy,fgz,phase,event,asm_ok\r\n",
	                alpha_accel_percent, alpha_gyro_percent, sample_period_ms);
}

int DataLogger_FormatSample(char *buffer, size_t size, const DataLogSample *sample)
{
	return snprintf(buffer, size,
	                "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
	                (unsigned long)sample->t_ms,
	                sample->accel_raw_mg[0], sample->accel_raw_mg[1], sample->accel_raw_mg[2],
	                sample->gyro_raw_mdps[0], sample->gyro_raw_mdps[1], sample->gyro_raw_mdps[2],
	                sample->accel_filt_mg[0], sample->accel_filt_mg[1], sample->accel_filt_mg[2],
	                sample->gyro_filt_mdps[0], sample->gyro_filt_mdps[1], sample->gyro_filt_mdps[2],
	                (int)sample->phase, (int)sample->event,
	                sample->asm_matches_c ? 1 : 0);
}

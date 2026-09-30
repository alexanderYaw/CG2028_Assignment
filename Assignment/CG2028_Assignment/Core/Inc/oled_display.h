/******************************************************************************
 * @file           : oled_display.h
 * @brief          : CG2028 Assignment - SSD1306 OLED status display
 *
 * Enhancement: shows the wearable's state on a 128x64 I2C OLED, counts down
 * the seconds until the long-lie alarm escalates, and shows a simulated
 * emergency call once it does.
 *
 * Wiring - the display sits on I2C1, which is free; the onboard accelerometer
 * and gyroscope are on I2C2 and are not touched:
 *
 *   OLED SCL -> PB8  (Arduino header D15)
 *   OLED SDA -> PB9  (Arduino header D14)
 *   OLED VCC -> 3V3
 *   OLED GND -> GND
 *
 * Timing: a full 128x64 refresh is about 11 ms at 100 kHz, which is longer
 * than one 20 ms sample period. The driver therefore keeps an 8-row text
 * buffer and pushes at most one changed row per call, and never pushes while
 * the detector is waiting for an impact, so sampling is not delayed at the
 * one moment that matters.
 ******************************************************************************/

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#include "fall_detector.h"

/* The Grove OLED 0.96" carries an SSD1315, command-compatible with the
 * SSD1306. Its silkscreen says 0x78, which is the 8-bit form of 7-bit 0x3C -
 * the same number the HAL wants here. Boards re-strapped to 0x3D need 0x7A. */
#define OLED_I2C_ADDRESS        0x78

/* Text grid: 5x7 font plus one spacing column = 6 px per character. */
#define OLED_COLS               21
#define OLED_ROWS                8

/* How often a changed row may be pushed to the panel. */
#define OLED_FLUSH_INTERVAL_MS  60
#define OLED_EVAL_FLUSH_MS     200   /* slower while a fall is being judged */

/* Returns false if the panel does not acknowledge, e.g. it is not wired up.
 * The rest of the system runs normally either way. */
bool OLED_Init(void);

/* Call once per sample from the main loop. Composes the screen for the
 * current phase and pushes at most one changed row. */
void OLED_Update(const FallDetector *detector, uint32_t now_ms);

/* Seconds left before a confirmed fall escalates to the long-lie alarm.
 * Zero in any other phase. */
uint32_t OLED_SecondsUntilLongLie(const FallDetector *detector, uint32_t now_ms);

#endif /* OLED_DISPLAY_H */

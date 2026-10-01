/* Renders the OLED screens on a PC, so the layout and the countdown can be
 * checked without the panel. It includes the driver with OLED_HOST_PREVIEW
 * defined, which compiles the screen composition and the font but leaves out
 * every I2C call, so what you see here is what the board will draw.
 *
 * Build and run from the repository root:
 *   gcc -O2 -IAssignment/CG2028_Assignment/Core/Inc \
 *       Part2_Simulation/oled_preview.c -o oled_preview && ./oled_preview
 */

#define OLED_HOST_PREVIEW
#include "../Assignment/CG2028_Assignment/Core/Src/oled_display.c"

#include <stdio.h>
#include <string.h>

static void render(const char *caption)
{
    printf("%s\n    +", caption);
    for (int i = 0; i < OLED_COLS * 6; i++) putchar('-');
    printf("+\n");

    for (int row = 0; row < OLED_ROWS; row++)
    {
        for (int pixel_row = 0; pixel_row < 8; pixel_row++)
        {
            printf("    |");
            for (int col = 0; col < OLED_COLS; col++)
            {
                unsigned char c = (unsigned char)text_rows[row][col];
                if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) c = ' ';
                const uint8_t *glyph = font5x7[c - FONT_FIRST_CHAR];
                for (int i = 0; i < FONT_WIDTH; i++)
                    putchar((glyph[i] >> pixel_row) & 1 ? '#' : ' ');
                putchar(' ');
            }
            printf("|\n");
        }
    }
    printf("    +");
    for (int i = 0; i < OLED_COLS * 6; i++) putchar('-');
    printf("+\n\n");
}

static void clear_buffer(void)
{
    memset(text_rows, 0, sizeof(text_rows));
    memset(row_dirty, 0, sizeof(row_dirty));
}

int main(void)
{
    FallDetector d;

    /* Normal monitoring, 1.01 g, 4 degrees from the reference posture */
    memset(&d, 0, sizeof(d));
    d.phase = FALL_PHASE_MONITORING;
    d.accel_mag_mg = 1012;
    d.tilt_deg = 4;
    clear_buffer(); compose_screen(&d, 5000); render("MONITORING");

    memset(&d, 0, sizeof(d));
    d.phase = FALL_PHASE_POST_IMPACT;
    d.phase_start_ms = 10000;
    clear_buffer(); compose_screen(&d, 11000); render("CHECKING A POSSIBLE FALL");

    /* Countdown: alarm raised at t=20 s, shown 3 s later */
    memset(&d, 0, sizeof(d));
    d.phase = FALL_PHASE_ALARM;
    d.alarm_ms = 20000;
    d.phase_start_ms = 20000;
    clear_buffer(); compose_screen(&d, 23000); render("FALL ALARM, COUNTING DOWN");

    clear_buffer(); compose_screen(&d, 20000 + LONG_LIE_MS - 1500); render("FALL ALARM, 2 S LEFT");

    memset(&d, 0, sizeof(d));
    d.phase = FALL_PHASE_LONG_LIE;
    d.phase_start_ms = 50000;
    clear_buffer(); compose_screen(&d, 57000); render("LONG LIE, SIMULATED CALL");

    memset(&d, 0, sizeof(d));
    d.phase = FALL_PHASE_SOS;
    d.phase_start_ms = 70000;
    clear_buffer(); compose_screen(&d, 71000); render("MANUAL SOS");

    /* Countdown arithmetic, including the boundaries */
    memset(&d, 0, sizeof(d));
    d.phase = FALL_PHASE_ALARM;
    d.alarm_ms = 1000;
    printf("countdown check (LONG_LIE_MS = %u):\n", (unsigned)LONG_LIE_MS);
    const uint32_t offsets[] = {0, 1, 999, 1000, LONG_LIE_MS - 1000, LONG_LIE_MS - 1, LONG_LIE_MS};
    for (unsigned i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++)
        printf("  %6u ms after alarm -> %2u s\n", (unsigned)offsets[i],
               (unsigned)OLED_SecondsUntilLongLie(&d, 1000 + offsets[i]));

    d.phase = FALL_PHASE_MONITORING;
    printf("  while monitoring        -> %2u s (expected 0)\n",
           (unsigned)OLED_SecondsUntilLongLie(&d, 5000));
    return 0;
}

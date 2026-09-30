/******************************************************************************
 * @file           : oled_display.c
 * @brief          : CG2028 Assignment - SSD1306 OLED status display
 *
 * See oled_display.h for the wiring (I2C1 on PB8/PB9) and the timing rules.
 *
 * Structure:
 *   1. font table
 *   2. text buffer with per-row dirty flags
 *   3. SSD1306 commands over I2C1
 *   4. compose_screen(): pure function, turns detector state into 8 text rows
 *
 * compose_screen() touches no hardware, so Part2_Simulation/oled_preview.c can
 * render the screens on a PC to check the layout.
 ******************************************************************************/

#include "oled_display.h"

#include <stdio.h>
#include <string.h>

#ifndef OLED_HOST_PREVIEW
#include "main.h"
#endif

/* 5x7 font, ASCII 0x20-0x5A. One byte per column, bit 0 = top row.
 * Generated table; lowercase input is upshifted by the driver. */
#define FONT_FIRST_CHAR 0x20
#define FONT_LAST_CHAR  0x5A
#define FONT_WIDTH      5

static const uint8_t font5x7[][FONT_WIDTH] = {
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* space */
    {0x00, 0x00, 0x5F, 0x00, 0x00},  /* ! */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* " */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* # */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* $ */
    {0x63, 0x13, 0x08, 0x64, 0x63},  /* % */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* & */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* ' */
    {0x00, 0x00, 0x3E, 0x41, 0x00},  /* ( */
    {0x00, 0x41, 0x3E, 0x00, 0x00},  /* ) */
    {0x2A, 0x1C, 0x3E, 0x1C, 0x2A},  /* * */
    {0x08, 0x08, 0x3E, 0x08, 0x08},  /* + */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* , */
    {0x08, 0x08, 0x08, 0x08, 0x08},  /* - */
    {0x00, 0x00, 0x40, 0x00, 0x00},  /* . */
    {0x60, 0x10, 0x08, 0x04, 0x03},  /* / */
    {0x36, 0x59, 0x41, 0x4D, 0x36},  /* 0 */
    {0x00, 0x42, 0x7F, 0x40, 0x00},  /* 1 */
    {0x42, 0x61, 0x51, 0x49, 0x46},  /* 2 */
    {0x21, 0x41, 0x45, 0x4B, 0x31},  /* 3 */
    {0x18, 0x14, 0x12, 0x7F, 0x10},  /* 4 */
    {0x27, 0x45, 0x45, 0x45, 0x39},  /* 5 */
    {0x3C, 0x4A, 0x49, 0x49, 0x30},  /* 6 */
    {0x01, 0x01, 0x79, 0x05, 0x03},  /* 7 */
    {0x36, 0x49, 0x49, 0x49, 0x36},  /* 8 */
    {0x06, 0x49, 0x49, 0x29, 0x1E},  /* 9 */
    {0x00, 0x00, 0x22, 0x00, 0x00},  /* : */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* ; */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* < */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* = */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* > */
    {0x02, 0x01, 0x51, 0x09, 0x06},  /* ? */
    {0x00, 0x00, 0x00, 0x00, 0x00},  /* @ */
    {0x7C, 0x12, 0x11, 0x12, 0x7C},  /* A */
    {0x7F, 0x49, 0x49, 0x49, 0x36},  /* B */
    {0x3E, 0x41, 0x41, 0x41, 0x22},  /* C */
    {0x7F, 0x41, 0x41, 0x41, 0x3E},  /* D */
    {0x7F, 0x49, 0x49, 0x49, 0x41},  /* E */
    {0x7F, 0x09, 0x09, 0x09, 0x01},  /* F */
    {0x3E, 0x41, 0x41, 0x49, 0x3A},  /* G */
    {0x7F, 0x08, 0x08, 0x08, 0x7F},  /* H */
    {0x00, 0x41, 0x7F, 0x41, 0x00},  /* I */
    {0x20, 0x40, 0x41, 0x3F, 0x01},  /* J */
    {0x7F, 0x08, 0x14, 0x22, 0x41},  /* K */
    {0x7F, 0x40, 0x40, 0x40, 0x40},  /* L */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},  /* M */
    {0x7F, 0x02, 0x04, 0x08, 0x7F},  /* N */
    {0x3E, 0x41, 0x41, 0x41, 0x3E},  /* O */
    {0x7F, 0x09, 0x09, 0x09, 0x06},  /* P */
    {0x3E, 0x41, 0x51, 0x21, 0x5E},  /* Q */
    {0x7F, 0x09, 0x19, 0x29, 0x46},  /* R */
    {0x46, 0x49, 0x49, 0x49, 0x31},  /* S */
    {0x01, 0x01, 0x7F, 0x01, 0x01},  /* T */
    {0x3F, 0x40, 0x40, 0x40, 0x3F},  /* U */
    {0x1F, 0x20, 0x40, 0x20, 0x1F},  /* V */
    {0x7F, 0x20, 0x18, 0x20, 0x7F},  /* W */
    {0x63, 0x14, 0x08, 0x14, 0x63},  /* X */
    {0x03, 0x04, 0x78, 0x04, 0x03},  /* Y */
    {0x61, 0x51, 0x49, 0x45, 0x43},  /* Z */
};

/*--------------------------- Text buffer ------------------------------------*/

static char text_rows[OLED_ROWS][OLED_COLS + 1];
static bool row_dirty[OLED_ROWS];

static void set_row(int row, const char *text)
{
    if (row < 0 || row >= OLED_ROWS)
    {
        return;
    }
    char padded[OLED_COLS + 1];
    size_t i = 0;

    while (i < (size_t)OLED_COLS && text[i] != '\0')
    {
        char c = text[i];
        if (c >= 'a' && c <= 'z')
        {
            c = (char)(c - 'a' + 'A');        /* the font is uppercase only */
        }
        padded[i] = c;
        i++;
    }
    while (i < (size_t)OLED_COLS)
    {
        padded[i++] = ' ';
    }
    padded[OLED_COLS] = '\0';

    if (strcmp(padded, text_rows[row]) != 0)
    {
        strcpy(text_rows[row], padded);
        row_dirty[row] = true;
    }
}

/*--------------------------- SSD1306 over I2C1 ------------------------------*/
#ifndef OLED_HOST_PREVIEW

static I2C_HandleTypeDef hi2c_oled;
static bool oled_ready;

static bool oled_write(uint8_t control, const uint8_t *payload, uint16_t length)
{
    uint8_t buffer[1 + OLED_COLS * FONT_WIDTH + 8];

    if ((size_t)length + 1U > sizeof(buffer))
    {
        return false;
    }
    buffer[0] = control;                       /* 0x00 command, 0x40 data */
    memcpy(&buffer[1], payload, length);

    return HAL_I2C_Master_Transmit(&hi2c_oled, OLED_I2C_ADDRESS, buffer,
                                   (uint16_t)(length + 1), 100) == HAL_OK;
}

static bool oled_command(uint8_t command)
{
    return oled_write(0x00, &command, 1);
}

static bool i2c1_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;        /* SCL = PB8, SDA = PB9 */
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    hi2c_oled.Instance = I2C1;
    /* Same timing word the BSP uses for the sensor bus: ~100 kHz with the
     * 80 MHz PCLK1 that SystemClock_Config sets up. */
    hi2c_oled.Init.Timing = 0x00702681;
    hi2c_oled.Init.OwnAddress1 = 0;
    hi2c_oled.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c_oled.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c_oled.Init.OwnAddress2 = 0;
    hi2c_oled.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c_oled.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c_oled) != HAL_OK)
    {
        return false;
    }
    return HAL_I2CEx_ConfigAnalogFilter(&hi2c_oled, I2C_ANALOGFILTER_ENABLE) == HAL_OK;
}

/* Push one text row as one SSD1306 page (8 pixel rows). */
static void flush_row(int row)
{
    uint8_t pixels[OLED_COLS * FONT_WIDTH + OLED_COLS];
    uint16_t index = 0;

    for (int col = 0; col < OLED_COLS; col++)
    {
        unsigned char c = (unsigned char)text_rows[row][col];
        if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR)
        {
            c = ' ';
        }
        const uint8_t *glyph = font5x7[c - FONT_FIRST_CHAR];
        for (int i = 0; i < FONT_WIDTH; i++)
        {
            pixels[index++] = glyph[i];
        }
        pixels[index++] = 0x00;                /* spacing column */
    }

    /* Page addressing mode: select the page, reset the column, send pixels. */
    if (!oled_command((uint8_t)(0xB0 | (row & 0x07))) ||
        !oled_command(0x00) ||                 /* column low nibble  = 0 */
        !oled_command(0x10))                   /* column high nibble = 0 */
    {
        return;
    }
    oled_write(0x40, pixels, index);
}

bool OLED_Init(void)
{
    static const uint8_t init_sequence[] = {
        0xAE,               /* display off                        */
        0x20, 0x02,         /* page addressing mode               */
        0xA1,               /* segment remap (0xA0 if mirrored)   */
        0xC8,               /* COM scan descending (0xC0 to flip) */
        0xA8, 0x3F,         /* multiplex ratio, 64 rows           */
        0xD3, 0x00,         /* display offset                     */
        0x40,               /* start line 0                       */
        0xDA, 0x12,         /* COM pins: alternating              */
        0xD5, 0x80,         /* clock divide                       */
        0xD9, 0xF1,         /* pre-charge                         */
        0xDB, 0x40,         /* VCOMH                              */
        0x81, 0x9F,         /* contrast                           */
        0xA4,               /* follow RAM contents                */
        0xA6,               /* not inverted                       */
        0x8D, 0x14,         /* charge pump on                     */
        0xAF                /* display on                         */
    };

    memset(text_rows, 0, sizeof(text_rows));
    memset(row_dirty, 0, sizeof(row_dirty));
    oled_ready = false;

    if (!i2c1_init())
    {
        return false;
    }
    /* If nothing acknowledges, the panel is absent or miswired. */
    if (HAL_I2C_IsDeviceReady(&hi2c_oled, OLED_I2C_ADDRESS, 3, 100) != HAL_OK)
    {
        return false;
    }
    for (size_t i = 0; i < sizeof(init_sequence); i++)
    {
        if (!oled_command(init_sequence[i]))
        {
            return false;
        }
    }

    oled_ready = true;

    /* Blank every page, then draw the splash. */
    for (int row = 0; row < OLED_ROWS; row++)
    {
        set_row(row, "");
        flush_row(row);
        row_dirty[row] = false;
    }
    set_row(0, "  ELDERCARE  ");
    set_row(2, " SAFETY COMPANION");
    set_row(5, " STARTING UP...");
    for (int row = 0; row < OLED_ROWS; row++)
    {
        if (row_dirty[row])
        {
            flush_row(row);
            row_dirty[row] = false;
        }
    }
    return true;
}

#endif /* !OLED_HOST_PREVIEW */

/*--------------------------- Screen composition -----------------------------*/

uint32_t OLED_SecondsUntilLongLie(const FallDetector *detector, uint32_t now_ms)
{
    if (detector->phase != FALL_PHASE_ALARM)
    {
        return 0;
    }
    uint32_t elapsed_ms = now_ms - detector->alarm_ms;
    if (elapsed_ms >= LONG_LIE_MS)
    {
        return 0;
    }
    /* Round up so the display shows "1 S" for the final second. */
    return (LONG_LIE_MS - elapsed_ms + 999U) / 1000U;
}

/* Pure function: current state in, eight rows of text out. */
static void compose_screen(const FallDetector *detector, uint32_t now_ms)
{
    char line[40];

    switch (detector->phase)
    {
    case FALL_PHASE_MONITORING:
        set_row(0, "ELDERCARE   MONITOR");
        set_row(1, "");
        set_row(2, "STATUS: ALL OK");
        snprintf(line, sizeof(line), "MOTION: %d.%02d G",
                 detector->accel_mag_mg / 1000, (detector->accel_mag_mg % 1000) / 10);
        set_row(3, line);
        snprintf(line, sizeof(line), "POSTURE: %d DEG", detector->tilt_deg);
        set_row(4, line);
        set_row(5, "");
        set_row(6, "HOLD BUTTON 2S");
        set_row(7, "FOR SOS");
        break;

    case FALL_PHASE_AWAIT_IMPACT:
    case FALL_PHASE_POST_IMPACT:
        set_row(0, "ELDERCARE   MONITOR");
        set_row(1, "");
        set_row(2, "SUDDEN MOVEMENT");
        set_row(3, "CHECKING...");
        set_row(4, "");
        set_row(5, "");
        set_row(6, "");
        set_row(7, "");
        break;

    case FALL_PHASE_ALARM:
    {
        uint32_t seconds = OLED_SecondsUntilLongLie(detector, now_ms);

        set_row(0, "!! FALL DETECTED !!");
        set_row(1, "");
        set_row(2, "CALLING HELP IN");
        snprintf(line, sizeof(line), "      %lu S", (unsigned long)seconds);
        set_row(3, line);
        set_row(4, "");
        set_row(5, "PRESS BUTTON");
        set_row(6, "IF YOU ARE OK");
        set_row(7, "");
        break;
    }

    case FALL_PHASE_LONG_LIE:
        set_row(0, "!! NO RESPONSE !!");
        set_row(1, "");
        set_row(2, "CALLING 995...");
        set_row(3, "(SIMULATED DEMO)");
        set_row(4, "");
        set_row(5, "HELP IS ON THE WAY");
        set_row(6, "STAY STILL");
        snprintf(line, sizeof(line), "ALARM FOR %lu S",
                 (unsigned long)((now_ms - detector->phase_start_ms) / 1000U));
        set_row(7, line);
        break;

    case FALL_PHASE_SOS:
        set_row(0, "!!! SOS ACTIVE !!!");
        set_row(1, "");
        set_row(2, "CALLING 995...");
        set_row(3, "(SIMULATED DEMO)");
        set_row(4, "");
        set_row(5, "PRESS BUTTON");
        set_row(6, "TO CANCEL");
        set_row(7, "");
        break;

    default:
        break;
    }
}

#ifndef OLED_HOST_PREVIEW

void OLED_Update(const FallDetector *detector, uint32_t now_ms)
{
    static uint32_t last_flush_ms = 0;
    static uint32_t last_compose_ms = 0;

    if (!oled_ready)
    {
        return;
    }

    /* Recomposing is cheap, but there is no point doing it every sample. */
    if ((now_ms - last_compose_ms) >= 100U)
    {
        last_compose_ms = now_ms;
        compose_screen(detector, now_ms);
    }

    /* Never transmit while waiting for an impact: an 11 ms transfer could
     * push the next sample past the peak. */
    if (detector->phase == FALL_PHASE_AWAIT_IMPACT)
    {
        return;
    }

    uint32_t interval_ms = (detector->phase == FALL_PHASE_POST_IMPACT)
                         ? OLED_EVAL_FLUSH_MS : OLED_FLUSH_INTERVAL_MS;
    if ((now_ms - last_flush_ms) < interval_ms)
    {
        return;
    }

    for (int row = 0; row < OLED_ROWS; row++)
    {
        if (row_dirty[row])
        {
            flush_row(row);
            row_dirty[row] = false;
            last_flush_ms = now_ms;
            return;                 /* one row per call keeps the cost bounded */
        }
    }
}

#endif /* !OLED_HOST_PREVIEW */

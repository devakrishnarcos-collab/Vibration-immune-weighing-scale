/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Vibration Immune Smart Weighing Scale (calibrated, kg)
  *
  * Current stage:
  *   - HX711 at 80 SPS (RATE pin tied HIGH on the HX711 board)
  *   - 63-tap FIR low-pass, fc = 3 Hz, applied to the raw readings
  *   - Boot-time tare (platform must be empty at power-up)
  *   - Calibrated weight in kg (CAL_COUNTS_PER_KG, from 81 g / 134 g / 228 g)
  *   - Fine tare: after power-on the zero is set from the first STABLE
  *     reading (see FINE_TARE_WARMUP_MS), under normal running conditions
  *   - Displayed weight is clamped at zero (never negative)
  *   - SSD1306 OLED shows everything on ONE page (no button needed):
  *       WT   : calibrated weight in kg
  *       RAW  : raw HX711 reading
  *       FILT : FIR-filtered reading
  *       NET  : filtered reading minus tare (counts)
  *   - UART prints RAW, FILTERED, NET and WEIGHT, plus measured SPS
  *   - Stable-weight detection: the OLED/UART weight is only updated while
  *     the reading is steady (STABLE); while it moves the last stable value
  *     is held and the OLED shows MOVING
  *   - Power button (Nucleo user button B1 by default): press once = ON,
  *     press again = OFF. OFF = OLED off, HX711 powered down, LD2 off.
  *     Every ON re-tares the scale (platform must be empty). The tare waits
  *     until the empty reading has settled (3 steady 1 s averages).
  *   - The real HX711 sample rate is measured during the boot tare and used
  *     to design the FIR filter and the FFT frequency axis
  *   - FFT analysis (256-point, Hann) of the RAW and FILTERED signals every
  *     3.2 s: strongest line and strongest line above the FIR cutoff are
  *     printed on UART; the RAW peak (row 1) and the FILTERED peak (row 7)
  *     are shown on the OLED
  *
  * Hardware: NUCLEO-F411RE, SSD1306 128x64 I2C OLED, HX711, Load Cell
  *
  * OLED  : SDA -> D14 / PB9, SCL -> D15 / PB8, Address = 0x3C, I2C 400 kHz
  * HX711 : DATA -> A0 / PA0, SCK -> A1 / PA1, RATE pin -> 3.3 V (80 SPS)
  * UART  : USART2, TX -> PA2, RX -> PA3, 115200 baud
  *
  * IMPORTANT: the I2C HAL module must be enabled in the project:
  *   Core/Inc/stm32f4xx_hal_conf.h  ->  #define HAL_I2C_MODULE_ENABLED
  *   and stm32f4xx_hal_i2c.c / stm32f4xx_hal_i2c_ex.c must be in the build.
  **************************
  */
/* USER CODE END Header */

/* Make sure the I2C HAL module is switched on for this file even if
 * stm32f4xx_hal_conf.h still has it commented out. Must stay ABOVE main.h. */
#ifndef HAL_I2C_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#endif

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "fir_filter.h"
#include "fft_analysis.h"
#include "stable_weight.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* If main.h (CubeMX) already defines these, keep only one set. */
#ifndef HX711_SCK_Pin
#define HX711_SCK_GPIO_Port   GPIOA
#define HX711_SCK_Pin         GPIO_PIN_1
#endif

#ifndef HX711_DATA_Pin
#define HX711_DATA_GPIO_Port  GPIOA
#define HX711_DATA_Pin        GPIO_PIN_0
#endif

#define OLED_I2C_TIMEOUT_MS   100
#define OLED_COLUMNS          21
#define HX711_TIMEOUT_MS      1000

#define OLED_ADDR             (0x3C << 1)

/* Filter / timing settings */
#define HX711_FS_HZ           80.0f   /* RATE pin must be HIGH */
#define FIR_CUTOFF_HZ         3.0f
#define DISPLAY_PERIOD_MS     250
#define SPS_PERIOD_MS         1000

/* ---------------- Calibration ----------------
 * Least-squares fit (through zero) of net counts vs. mass for 81 g, 134 g
 * and 228 g, each measured against the average of the empty readings taken
 * before and after the mass:
 *     723.7 counts per gram  ->  723700 counts per kg
 * Residuals: +1.2 g (81 g), -0.2 g (134 g), -0.3 g (228 g).
 * The zero point is set at power-on (fine tare), so no fixed offset is needed.
 * If you recalibrate, only change CAL_COUNTS_PER_KG.
 */
#define CAL_COUNTS_PER_KG     723700.0f
#define ZERO_DEADBAND_G       2       /* weight below this shows 0.000 */

/* ---------------- Stable-weight detection ----------------
 * STABLE when (max - min) of the filtered net counts over the last ~1 s
 * is within STABLE_BAND_G. Raise it if an empty scale says MOVING too
 * often; lower it for a stricter test.
 * STABLE_HYST_G: a new stable weight replaces the shown one only if it
 * differs by MORE than this many grams (stops +/-1 g flicker).
 */
#define STABLE_BAND_G         4
#define STABLE_HYST_G         1

/* ---------------- Fine tare (zero) ----------------
 * Tare_Scale() waits for the empty reading to settle and gives the first
 * zero (it also measures the sample rate). The final zero is set once more
 * inside the run loop, while the OLED and UART run exactly as in normal
 * measurement: after FINE_TARE_WARMUP_MS, as soon as the reading is STABLE,
 * its 1 s average becomes the new zero. If it never becomes stable, the zero
 * is taken anyway after FINE_TARE_TIMEOUT_MS.
 * Keep the platform EMPTY and still during this time (OLED: TARING).
 */
#define FINE_TARE_WARMUP_MS   2000
#define FINE_TARE_TIMEOUT_MS  15000

/* ---------------- OLED single-page layout (page rows 0..7) ---------------- */
#define ROW_TITLE             0
#define ROW_WEIGHT            2
#define ROW_RAW               4
#define ROW_FILT              5
#define ROW_NET               6
#define ROW_FFT_FILT          7       /* FFT peak of the FILTERED signal */
#define ROW_ERROR             3
#define OLED_VALUE_LINES      4       /* WT, RAW, FILT, NET */

/* ---------------- Power button / switch ----------------
 * Default: the Nucleo blue user button B1 (PC13, active low).
 * External push button: wire it between PC0 (Arduino A5) and GND
 * (the internal pull-up is enabled, no resistor needed) and use
 *     #define PWR_BTN_PORT  GPIOC
 *     #define PWR_BTN_PIN   GPIO_PIN_0
 *
 * PWR_SWITCH_LATCHING = 0 : push button, every press toggles ON/OFF
 * PWR_SWITCH_LATCHING = 1 : latching switch, closed = ON, open = OFF
 * START_POWERED_ON        : 0 = start in standby (press to turn ON)
 */
#define PWR_BTN_PORT          GPIOC
#define PWR_BTN_PIN           GPIO_PIN_0
#define PWR_SWITCH_LATCHING   1
#define START_POWERED_ON      0
#define BUTTON_DEBOUNCE_MS    30
/* 1 = HX711 (and the load cell supply) is powered down in standby.
 * 0 = HX711 stays powered, so the load cell does not have to warm up
 *     again after every ON (try this if the zero drifts after power-on). */
#define HX711_POWER_DOWN_IN_OFF  1
#define ROW_FFT               1       /* FFT peak of the RAW signal */

/* USER CODE END Private defines */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* Microsecond delay (DWT cycle counter) */
static void DWT_Init(void);
static void Delay_us(uint32_t us);

/* OLED */
static void OLED_Command(uint8_t cmd);
static void OLED_Data(uint8_t *data, uint16_t size);
static void OLED_Init(void);
static void OLED_Clear(void);
static void OLED_SetCursor(uint8_t page, uint8_t column);
static void OLED_PrintLine(uint8_t page, const char *str);

/* HX711 */
static int HX711_IsReady(void);
static int HX711_Read(int32_t *value);
static float tare_offset = 0.0f;
static float measured_fs_hz = HX711_FS_HZ;   /* replaced by Tare_Scale() */
/* ---- Tare: wait until the empty reading has settled ----
 * The reading is averaged in blocks of TARE_BLOCK_SAMPLES (about 1 s). The
 * tare is accepted when TARE_BLOCKS_NEEDED consecutive block averages agree
 * within TARE_TOLERANCE_G. If that does not happen within TARE_TIMEOUT_MS the
 * last block average is used and a warning is printed. */
#define TARE_BLOCK_SAMPLES    80
#define TARE_BLOCKS_NEEDED    3
#define TARE_TOLERANCE_G      1.5f
#define TARE_TIMEOUT_MS       15000

/* Returns 1 = tare taken on a settled signal,
 *         2 = tare taken but the signal never settled (timeout),
 *         0 = HX711 read failed. */
static int Tare_Scale(float *offset)
{
    int32_t  raw;
    float    means[TARE_BLOCKS_NEEDED];
    uint8_t  n_means = 0;
    float    last_mean = 0.0f;
    float    tol = TARE_TOLERANCE_G * CAL_COUNTS_PER_KG / 1000.0f;
    uint32_t t_start;

    HAL_Delay(1000);                    /* warm-up before the first block */
    t_start = HAL_GetTick();

    while ((HAL_GetTick() - t_start) < TARE_TIMEOUT_MS)
    {
        int64_t  sum = 0;
        uint32_t t0 = 0;

        for (int i = 0; i < TARE_BLOCK_SAMPLES; i++)
        {
            if (!HX711_Read(&raw))
            {
                return 0;
            }

            sum += raw;

            if (i == 0)
            {
                t0 = HAL_GetTick();     /* start timing after the first sample */
            }
        }

        /* Real sample rate from this block: (N - 1) intervals since t0 */
        uint32_t dt_ms = HAL_GetTick() - t0;

        if (dt_ms > 0)
        {
            float fs = (float)(TARE_BLOCK_SAMPLES - 1) * 1000.0f / (float)dt_ms;

            if (fs > 60.0f && fs < 100.0f)          /* sanity check */
            {
                measured_fs_hz = fs;
            }
        }

        last_mean = (float)sum / (float)TARE_BLOCK_SAMPLES;

        /* Keep the last TARE_BLOCKS_NEEDED block averages */
        if (n_means < TARE_BLOCKS_NEEDED)
        {
            means[n_means++] = last_mean;
        }
        else
        {
            for (int k = 0; k < TARE_BLOCKS_NEEDED - 1; k++)
            {
                means[k] = means[k + 1];
            }
            means[TARE_BLOCKS_NEEDED - 1] = last_mean;
        }

        if (n_means == TARE_BLOCKS_NEEDED)
        {
            float mn = means[0];
            float mx = means[0];
            float avg = 0.0f;

            for (int k = 0; k < TARE_BLOCKS_NEEDED; k++)
            {
                avg += means[k];
                if (means[k] < mn) mn = means[k];
                if (means[k] > mx) mx = means[k];
            }

            if ((mx - mn) <= tol)
            {
                *offset = avg / (float)TARE_BLOCKS_NEEDED;
                return 1;               /* settled */
            }
        }
    }

    *offset = last_mean;                /* timeout: best effort */
    return 2;
}

/*
 * Convert net counts to grams (integer), clamped at zero.
 * Anything below ZERO_DEADBAND_G (including all negative values)
 * is shown as 0, so the weight can never go below zero.
 * Integer maths only: %f is not available with newlib-nano by default.
 * Resolution is 1 g (the signal noise is about +/-2 g anyway).
 */
static long Weight_Grams(float net_counts)
{
    float kg = net_counts / CAL_COUNTS_PER_KG;
    long  g  = lroundf(kg * 1000.0f);

    if (g < ZERO_DEADBAND_G)
    {
        g = 0;
    }
    return g;
}

/* Text weight like "0.228 KG" (weight is never negative) */
static void Format_Weight(char *out, size_t size, long grams)
{
    if (grams < 0)
    {
        grams = 0;
    }

    snprintf(out, size, "%ld.%03ld KG", grams / 1000, grams % 1000);
}

/* UART */
static void UART_Print(const char *str)
{
    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)str,
        (uint16_t)strlen(str),
        HAL_MAX_DELAY
    );
}

/* Print one FFT result line (integers only: no %f with newlib-nano).
 * Frequencies in 0.01 Hz steps, amplitudes in milligrams (mg). */
static void FFT_Report(const char *tag, const FFT_Result_t *r)
{
    char line[96];
    const float mg_per_count = 1.0e6f / CAL_COUNTS_PER_KG;

    long pk_c  = (long)(r->peak_hz  * 100.0f + 0.5f);
    long vb_c  = (long)(r->vib_hz   * 100.0f + 0.5f);
    long pk_mg = (long)(r->peak_amp * mg_per_count + 0.5f);
    long vb_mg = (long)(r->vib_amp  * mg_per_count + 0.5f);

    snprintf(line, sizeof(line),
             "FFT %s PEAK %ld.%02ld HZ %ld MG | >%d HZ: %ld.%02ld HZ %ld MG\r\n",
             tag,
             pk_c / 100, pk_c % 100, pk_mg,
             (int)FIR_CUTOFF_HZ,
             vb_c / 100, vb_c % 100, vb_mg);

    UART_Print(line);
}

/* OLED text for one FFT result, e.g. "RAW PK 12.28HZ 919MG" */
static void FFT_OledText(char *out, size_t size, const char *tag,
                         const FFT_Result_t *r)
{
    long pk_c  = (long)(r->peak_hz  * 100.0f + 0.5f);
    long pk_mg = (long)(r->peak_amp * (1.0e6f / CAL_COUNTS_PER_KG) + 0.5f);

    snprintf(out, size, "%s PK %ld.%02ldHZ %ldMG",
             tag, pk_c / 100, pk_c % 100, pk_mg);
}

/* Font (uppercase letters, digits, space : - .) */
static const uint8_t Font5x7[][5] =
{
    [32] = {0x00, 0x00, 0x00, 0x00, 0x00}, /* space */
    [45] = {0x08, 0x08, 0x08, 0x08, 0x08}, /* - */
    [46] = {0x00, 0x60, 0x60, 0x00, 0x00}, /* . */
    [58] = {0x00, 0x36, 0x36, 0x00, 0x00}, /* : */

    [48] = {0x3E, 0x51, 0x49, 0x45, 0x3E},
    [49] = {0x00, 0x42, 0x7F, 0x40, 0x00},
    [50] = {0x42, 0x61, 0x51, 0x49, 0x46},
    [51] = {0x21, 0x41, 0x45, 0x4B, 0x31},
    [52] = {0x18, 0x14, 0x12, 0x7F, 0x10},
    [53] = {0x27, 0x45, 0x45, 0x45, 0x39},
    [54] = {0x3C, 0x4A, 0x49, 0x49, 0x30},
    [55] = {0x01, 0x71, 0x09, 0x05, 0x03},
    [56] = {0x36, 0x49, 0x49, 0x49, 0x36},
    [57] = {0x06, 0x49, 0x49, 0x29, 0x1E},

    [65] = {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* A */
    [66] = {0x7F, 0x49, 0x49, 0x49, 0x36}, /* B */
    [67] = {0x3E, 0x41, 0x41, 0x41, 0x22}, /* C */
    [68] = {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* D */
    [69] = {0x7F, 0x49, 0x49, 0x49, 0x41}, /* E */
    [70] = {0x7F, 0x09, 0x09, 0x09, 0x01}, /* F */
    [71] = {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* G */
    [72] = {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* H */
    [73] = {0x00, 0x41, 0x7F, 0x41, 0x00}, /* I */
    [74] = {0x20, 0x40, 0x41, 0x3F, 0x01}, /* J */
    [75] = {0x7F, 0x08, 0x14, 0x22, 0x41}, /* K */
    [76] = {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L */
    [77] = {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* M */
    [78] = {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* N */
    [79] = {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O */
    [80] = {0x7F, 0x09, 0x09, 0x09, 0x06}, /* P */
    [81] = {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* Q */
    [82] = {0x7F, 0x09, 0x19, 0x29, 0x46}, /* R */
    [83] = {0x46, 0x49, 0x49, 0x49, 0x31}, /* S */
    [84] = {0x01, 0x01, 0x7F, 0x01, 0x01}, /* T */
    [85] = {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* U */
    [86] = {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* V */
    [87] = {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* W */
    [88] = {0x63, 0x14, 0x08, 0x14, 0x63}, /* X */
    [89] = {0x07, 0x08, 0x70, 0x08, 0x07}, /* Y */
    [90] = {0x61, 0x51, 0x49, 0x45, 0x43}  /* Z */
};

/* ========================================================================= */
/*                         MICROSECOND DELAY                                 */
/* ========================================================================= */

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

/* ========================================================================= */
/*                         OLED FUNCTIONS                                    */
/* ========================================================================= */

static void OLED_Command(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};

    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, data, 2, OLED_I2C_TIMEOUT_MS);
}

static void OLED_Data(uint8_t *data, uint16_t size)
{
    uint8_t buffer[129];

    if (size > 128)
        size = 128;

    buffer[0] = 0x40;
    memcpy(&buffer[1], data, size);

    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, buffer, size + 1,
                            OLED_I2C_TIMEOUT_MS);
}

static void OLED_Init(void)
{
    HAL_Delay(100);

    OLED_Command(0xAE);                       /* Display OFF */
    OLED_Command(0xD5); OLED_Command(0x80);   /* Clock divide */
    OLED_Command(0xA8); OLED_Command(0x3F);   /* Multiplex 1/64 */
    OLED_Command(0xD3); OLED_Command(0x00);   /* Display offset */
    OLED_Command(0x40);                       /* Start line 0 */
    OLED_Command(0x8D); OLED_Command(0x14);   /* Charge pump ON */
    OLED_Command(0x20); OLED_Command(0x02);   /* PAGE addressing (required by
                                                 OLED_SetCursor B0/00/10 cmds) */
    OLED_Command(0xA1);                       /* Segment remap */
    OLED_Command(0xC8);                       /* COM scan reversed */
    OLED_Command(0xDA); OLED_Command(0x12);   /* COM pins */
    OLED_Command(0x81); OLED_Command(0x7F);   /* Contrast */
    OLED_Command(0xD9); OLED_Command(0xF1);   /* Pre-charge */
    OLED_Command(0xDB); OLED_Command(0x40);   /* VCOMH */
    OLED_Command(0xA4);                       /* Resume from RAM */
    OLED_Command(0xA6);                       /* Normal display */
    OLED_Command(0xAF);                       /* Display ON */

    OLED_Clear();
}

static void OLED_Clear(void)
{
    uint8_t zero[128];

    memset(zero, 0x00, sizeof(zero));

    for (uint8_t page = 0; page < 8; page++)
    {
        OLED_SetCursor(page, 0);
        OLED_Data(zero, 128);
    }
}

static void OLED_SetCursor(uint8_t page, uint8_t column)
{
    OLED_Command(0xB0 | (page & 0x07));
    OLED_Command(0x00 | (column & 0x0F));
    OLED_Command(0x10 | ((column >> 4) & 0x0F));
}

/*
 * Print one text line (whole page row) in a SINGLE I2C transfer.
 * The row is built in RAM first, padded with blanks, then sent at once.
 * This is ~10x faster than sending character by character, so the OLED
 * update cannot make the main loop miss HX711 samples (12.5 ms period).
 */
static void OLED_PrintLine(uint8_t page, const char *str)
{
    uint8_t row[128];
    uint8_t col = 0;
    uint8_t n = 0;

    memset(row, 0x00, sizeof(row));

    while (*str && n < OLED_COLUMNS)
    {
        char c = *str++;

        /* The font only has uppercase: convert lowercase letters */
        if (c >= 'a' && c <= 'z')
            c = (char)(c - 32);

        if ((uint8_t)c < 32 || (uint8_t)c > 90)
            c = ' ';

        memcpy(&row[col], Font5x7[(uint8_t)c], 5);   /* 5 columns + 1 blank */
        col += 6;
        n++;
    }

    OLED_SetCursor(page, 0);
    OLED_Data(row, 128);
}

/* ========================================================================= */
/*                         HX711 FUNCTIONS                                   */
/* ========================================================================= */

/* HX711 DATA pin goes LOW when a conversion is ready. */
static int HX711_IsReady(void)
{
    return (HAL_GPIO_ReadPin(HX711_DATA_GPIO_Port, HX711_DATA_Pin)
            == GPIO_PIN_RESET);
}

/*
 * Read one 24-bit conversion (Channel A, gain 128).
 * Returns 1 on success, 0 on timeout.
 *  - ~1 us clock high/low times (HX711 minimum is 0.2 us).
 *  - Interrupts are disabled during the 25 pulses so SCK can never stay
 *    high for >60 us (which would put the HX711 into power-down).
 */
static int HX711_Read(int32_t *value)
{
    uint32_t data = 0;
    uint32_t start_time = HAL_GetTick();

    while (!HX711_IsReady())
    {
        if ((HAL_GetTick() - start_time) >= HX711_TIMEOUT_MS)
        {
            return 0;
        }
    }

    __disable_irq();

    for (uint8_t i = 0; i < 24; i++)
    {
        HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
        Delay_us(1);

        data <<= 1;
        if (HAL_GPIO_ReadPin(HX711_DATA_GPIO_Port, HX711_DATA_Pin)
            == GPIO_PIN_SET)
        {
            data |= 1;
        }

        HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
        Delay_us(1);
    }

    /* 25th pulse: select Channel A, Gain 128 for the next conversion */
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
    Delay_us(1);
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
    Delay_us(1);

    __enable_irq();

    /* 24-bit two's complement -> signed 32-bit */
    if (data & 0x800000)
    {
        data |= 0xFF000000;
    }

    *value = (int32_t)data;
    return 1;
}

/* ========================================================================= */
/*                         POWER ON / OFF                                    */
/* ========================================================================= */

/* HX711: SCK held HIGH for more than 60 us = power-down, SCK LOW = power-up */
static void HX711_PowerDown(void)
{
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
    Delay_us(100);
}

static void HX711_PowerUp(void)
{
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
}

/* SSD1306: display off + charge pump off (about 10 uA), RAM is kept */
static void OLED_Sleep(void)
{
    OLED_Command(0xAE);                       /* Display OFF */
    OLED_Command(0x8D); OLED_Command(0x10);   /* Charge pump OFF */
}

static void OLED_Wake(void)
{
    OLED_Command(0x8D); OLED_Command(0x14);   /* Charge pump ON */
    OLED_Command(0xAF);                       /* Display ON */
    HAL_Delay(100);                           /* panel supply settling */
}

static void Scale_PowerOn(void)
{
    HX711_PowerUp();
    OLED_Clear();                             /* clear while the panel is off */
    OLED_Wake();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    HAL_Delay(500);                           /* HX711 settling after power-up */
}

static void Scale_PowerOff(void)
{
    OLED_Sleep();
#if HX711_POWER_DOWN_IN_OFF
    HX711_PowerDown();
#endif
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

/* Debounced power button. Call it often (every loop iteration).
 * pwr_btn_level : 1 while the button is pressed / the switch is closed.
 * Returns 1 exactly once per new (debounced) press. */
static uint8_t pwr_btn_level = 0;

static uint8_t PowerButton_Pressed(void)
{
    static uint8_t  last_raw = 0;
    static uint32_t t_raw = 0;

    uint8_t  raw = (HAL_GPIO_ReadPin(PWR_BTN_PORT, PWR_BTN_PIN)
                    == GPIO_PIN_RESET);
    uint32_t now = HAL_GetTick();

    if (raw != last_raw)
    {
        last_raw = raw;
        t_raw = now;                          /* level changed: restart timer */
    }

    if (raw != pwr_btn_level && (now - t_raw) >= BUTTON_DEBOUNCE_MS)
    {
        pwr_btn_level = raw;                  /* level stable long enough */
        return raw;                           /* 1 only for a new press */
    }

    return 0;
}

/* 1 = the user asked to turn the machine OFF (called in the run loop) */
static uint8_t Power_OffRequested(void)
{
    uint8_t pressed = PowerButton_Pressed();

#if PWR_SWITCH_LATCHING
    (void)pressed;
    return (uint8_t)!pwr_btn_level;
#else
    return pressed;
#endif
}

/* 1 = the user asked to turn the machine ON (called while it is off) */
static uint8_t Power_OnRequested(void)
{
    uint8_t pressed = PowerButton_Pressed();

#if PWR_SWITCH_LATCHING
    (void)pressed;
    return pwr_btn_level;
#else
    return pressed;
#endif
}

/* ========================================================================= */
/*                         MAIN                                              */
/* ========================================================================= */

int main(void)
{
    char     uart_buffer[120];
    char     oled_buffer[24];
    char     weight_text[24];
    int32_t  raw_value = 0;
    float    filtered = 0.0f;

    static FIR_t scale_fir;
    uint8_t  primed = 0;
    uint8_t  hx_error = 0;        /* 1 while "NO HX711 DATA" is on screen */

    /* Snapshot of the values shown on the OLED. The four lines are drawn
     * one per loop iteration (see below) so a display update never takes
     * longer than one HX711 sample period (12.5 ms). */
    uint8_t  oled_step = OLED_VALUE_LINES;   /* == idle, nothing to draw   */
    long     snap_weight_g = 0;
    uint8_t  snap_stable = 0;
    int32_t  snap_raw = 0;
    long     snap_filt = 0;
    long     snap_net = 0;

    /* FFT analysis of the raw and the filtered signal */
    static FFT_Analyzer_t fft_raw;
    static FFT_Analyzer_t fft_filt;
    FFT_Result_t fft_res;
    uint8_t  fft_show = 0;        /* 1 = new FFT text waiting for the OLED */
    char     fft_text[24];        /* RAW peak text    (OLED row 1) */
    uint8_t  fft_show_f = 0;      /* 1 = FILTERED text waiting for the OLED */
    char     fft_text_f[24];      /* FILTERED peak text (OLED row 7) */

    /* Stable-weight detection */
    static Stable_t stable;
    uint8_t  is_stable = 0;
    float    stable_mean = 0.0f;
    long     shown_g = 0;         /* weight shown on OLED / held while moving */
    char     shown_text[24];

    uint32_t last_display = 0;
    uint32_t last_sps = 0;
    uint32_t sample_count = 0;

    uint8_t  power_on = START_POWERED_ON;

    /* Fine tare */
    uint8_t  taring = 1;          /* 1 until the fine tare is done */
    uint8_t  snap_taring = 0;
    uint32_t tare_start = 0;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    DWT_Init();

    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR, 3, 100) == HAL_OK)
        UART_Print("OLED FOUND at address 0x3C\r\n");
    else
        UART_Print("OLED NOT FOUND at address 0x3C\r\n");

    OLED_Init();

    UART_Print("\r\n");
    UART_Print("========================================\r\n");
    UART_Print(" VIBRATION IMMUNE SMART WEIGHING SCALE\r\n");
    UART_Print(" HX711 80 SPS + FIR LOW-PASS (3 Hz)\r\n");
    UART_Print("========================================\r\n");

    if (!power_on)
    {
        Scale_PowerOff();
        UART_Print("STANDBY: press the power button to turn ON.\r\n");
    }

    while (1)     /* one pass = one ON session */
    {
        /* ===== OFF: wait for the power button ===== */
        while (!power_on)
        {
            if (Power_OnRequested())
            {
                power_on = 1;
            }
            else
            {
                HAL_Delay(10);
            }
        }

        /* ===== ON sequence ===== */
        Scale_PowerOn();          /* HX711 wake, OLED wake, LD2 on */

        UART_Print("\r\nPOWER ON\r\n");
        UART_Print("Remove all weight from the load cell.\r\n\r\n");

        /* Fresh start for every ON session */
        primed        = 0;
        hx_error      = 0;
        sample_count  = 0;
        oled_step     = OLED_VALUE_LINES;
        fft_show      = 0;
        fft_show_f    = 0;
        fft_raw.count = 0;   fft_raw.ready  = 0;
        fft_filt.count = 0;  fft_filt.ready = 0;
        shown_g       = 0;
        is_stable     = 0;
        stable_mean   = 0.0f;
        snap_weight_g = 0;   snap_stable = 0;   snap_taring = 0;
        snap_raw      = 0;   snap_filt = 0;   snap_net = 0;

        last_display = HAL_GetTick();
        last_sps     = HAL_GetTick();

        UART_Print("Keep scale empty and still...\r\n");
        OLED_PrintLine(3, "TARING...");

        int tare_result = Tare_Scale(&tare_offset);   /* 1 ok, 2 unsettled, 0 failed */

        if (tare_result != 0)
        {
            snprintf(uart_buffer, sizeof(uart_buffer),
                     "TARE OFFSET: %ld\r\n",
                     (long)tare_offset);

            UART_Print(uart_buffer);

            if (tare_result == 2)
            {
                UART_Print("TARE WARNING: signal never settled, zero may be off\r\n");
            }
        }
        else
        {
            UART_Print("TARE FAILED\r\n");
        }

        /* Design the filter and the FFT once, with the MEASURED sample rate
         * (the HX711 internal oscillator is not exactly 80 SPS) */
        FIR_DesignLowpass(&scale_fir, FIR_CUTOFF_HZ, measured_fs_hz);
        FFT_Init(measured_fs_hz);
        Stable_Init(&stable, (float)STABLE_BAND_G * CAL_COUNTS_PER_KG / 1000.0f);

        {
            long fs_c = (long)(measured_fs_hz * 100.0f + 0.5f);

            snprintf(uart_buffer, sizeof(uart_buffer),
                     "MEASURED FS: %ld.%02ld HZ\r\n", fs_c / 100, fs_c % 100);
            UART_Print(uart_buffer);
        }

        /* ---- Draw the static part of the single page once ---- */
        OLED_Clear();
        OLED_PrintLine(ROW_TITLE,  "SMART WEIGHING SCALE");

        taring     = 1;                    /* fine tare runs in the loop */
        tare_start = HAL_GetTick();

        /* ===== RUN: measure until the power button is pressed again ===== */
        while (power_on)
        {
            /* ---- Power button: a press (or opening the switch) = OFF ---- */
            if (Power_OffRequested())
            {
                power_on = 0;
                break;
            }

            /* ---- Read HX711 (blocks until next conversion, ~12.5 ms) ---- */
            if (HX711_Read(&raw_value) == 0)
            {
                UART_Print("HX711 read failed\r\n");
                OLED_PrintLine(ROW_ERROR, "NO HX711 DATA");
                hx_error = 1;
                continue;
            }

            if (hx_error)
            {
                OLED_PrintLine(ROW_ERROR, "");   /* clear the error message */
                hx_error = 0;
            }

            sample_count++;

            if (!primed)
            {
                FIR_Prime(&scale_fir, (float)raw_value);   /* no start-up ramp */
                primed = 1;
            }

            filtered = FIR_Update(&scale_fir, (float)raw_value);

            /* Feed both spectrum analysers (one sample each per loop) */
            FFT_Push(&fft_raw,  (float)raw_value);
            FFT_Push(&fft_filt, filtered);

            float net_counts = filtered - tare_offset;
            long  weight_g   = Weight_Grams(net_counts);   /* clamped >= 0, live */

            /* ---- Stable-weight detection: update the shown weight only when the
             *      last second of readings is steady ---- */
            is_stable = Stable_Update(&stable, net_counts, &stable_mean);

            /* ---- Fine tare: wait for a stable, warmed-up reading, then use
             *      its 1 s average as the new zero ---- */
            if (taring)
            {
                uint32_t el = HAL_GetTick() - tare_start;

                if ((is_stable && el >= FINE_TARE_WARMUP_MS) || el >= FINE_TARE_TIMEOUT_MS)
                {
                    tare_offset += stable_mean;          /* new zero */
                    Stable_Init(&stable,
                                (float)STABLE_BAND_G * CAL_COUNTS_PER_KG / 1000.0f);
                    is_stable = 0;
                    taring    = 0;
                    shown_g   = 0;

                    snprintf(uart_buffer, sizeof(uart_buffer),
                             "TARE DONE: %ld (after %lu ms)\r\n",
                             (long)tare_offset, (unsigned long)el);
                    UART_Print(uart_buffer);
                }
            }

            if (is_stable && !taring)
            {
                long mean_g = Weight_Grams(stable_mean);

                if (labs(mean_g - shown_g) > STABLE_HYST_G)
                {
                    shown_g = mean_g;
                }
            }

            /* ---- Measured sample rate (UART only): ~80 if RATE is HIGH ---- */
            if ((HAL_GetTick() - last_sps) >= SPS_PERIOD_MS)
            {
                uint32_t sps = (sample_count * 1000U) / (HAL_GetTick() - last_sps);
                sample_count = 0;
                last_sps = HAL_GetTick();

                snprintf(uart_buffer, sizeof(uart_buffer),
                         "SPS: %lu\r\n", (unsigned long)sps);
                UART_Print(uart_buffer);
            }

            /* ---- Slow output: UART line + start of an OLED refresh ---- */
            if ((HAL_GetTick() - last_display) >= DISPLAY_PERIOD_MS)
            {
                last_display = HAL_GetTick();

                Format_Weight(weight_text, sizeof(weight_text), weight_g);
                Format_Weight(shown_text, sizeof(shown_text), shown_g);

                snprintf(uart_buffer, sizeof(uart_buffer),
                         "RAW: %ld  FILT: %ld  NET: %ld  WT: %s  SHOWN: %s  %s\r\n",
                         (long)raw_value,
                         (long)filtered,
                         (long)net_counts,
                         weight_text,
                         shown_text,
                         taring ? "TARING" : (is_stable ? "STABLE" : "MOVING"));

                UART_Print(uart_buffer);

                /* Freeze the values for this OLED refresh and start drawing */
                snap_weight_g = shown_g;      /* held weight, not the live one */
                snap_stable   = is_stable;
                snap_taring   = taring;
                snap_raw      = raw_value;
                snap_filt     = (long)filtered;
                snap_net      = (long)net_counts;
                oled_step     = 0;
            }
            /* ---- FFT: at most ONE analysis per loop iteration, and never in the
             *      same iteration as the periodic UART line, so the loop still
             *      keeps up with the 12.5 ms HX711 sample period ---- */
            else if (fft_raw.ready)
            {
                FFT_Process(&fft_raw, FIR_CUTOFF_HZ, &fft_res);
                FFT_Report("RAW ", &fft_res);
                FFT_OledText(fft_text, sizeof(fft_text), "RAW", &fft_res);
                fft_show = 1;
            }
            else if (fft_filt.ready)
            {
                FFT_Process(&fft_filt, FIR_CUTOFF_HZ, &fft_res);
                FFT_Report("FILT", &fft_res);
                FFT_OledText(fft_text_f, sizeof(fft_text_f), "FILT", &fft_res);
                fft_show_f = 1;
            }

            /* ---- Draw ONE OLED line per loop iteration (keeps up with 80 SPS) ---- */
            if (oled_step < OLED_VALUE_LINES)
            {
                switch (oled_step)
                {
                case 0:
                    Format_Weight(weight_text, sizeof(weight_text), snap_weight_g);
                    snprintf(oled_buffer, sizeof(oled_buffer), "WT: %s %s",
                             weight_text,
                             snap_taring ? "TARING" : (snap_stable ? "STABLE" : "MOVING"));
                    OLED_PrintLine(ROW_WEIGHT, oled_buffer);
                    break;

                case 1:
                    snprintf(oled_buffer, sizeof(oled_buffer),
                             "RAW:  %ld", (long)snap_raw);
                    OLED_PrintLine(ROW_RAW, oled_buffer);
                    break;

                case 2:
                    snprintf(oled_buffer, sizeof(oled_buffer),
                             "FILT: %ld", snap_filt);
                    OLED_PrintLine(ROW_FILT, oled_buffer);
                    break;

                default:
                    snprintf(oled_buffer, sizeof(oled_buffer),
                             "NET:  %ld", snap_net);
                    OLED_PrintLine(ROW_NET, oled_buffer);
                    break;
                }

                oled_step++;
            }
            else if (fft_show)
            {
                OLED_PrintLine(ROW_FFT, fft_text);
                fft_show = 0;
            }
            else if (fft_show_f)
            {
                OLED_PrintLine(ROW_FFT_FILT, fft_text_f);
                fft_show_f = 0;
            }
        }

        /* ===== OFF sequence ===== */
        UART_Print("POWER OFF\r\n");
        OLED_Clear();
        OLED_PrintLine(3, "POWER OFF");
        HAL_Delay(600);
        Scale_PowerOff();         /* OLED off, HX711 power-down, LD2 off */
        UART_Print("STANDBY: press the power button to turn ON.\r\n");
    }
}

/* ========================================================================= */
/*                         SYSTEM CLOCK  (84 MHz from HSI)                   */
/* ========================================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 7;   /* 48 MHz, valid for USB */

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================= */
/*                         I2C1  (400 kHz fast mode)                         */
/* ========================================================================= */

/* Pin setup for PB8 (SCL) / PB9 (SDA), AF4 open-drain.
 * This overrides the weak HAL default. If you later let CubeMX generate
 * I2C1 (it creates this function in stm32f4xx_hal_msp.c), delete this copy
 * to avoid a "multiple definition" linker error. */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================= */
/*                         USART2                                            */
/* ========================================================================= */

/* NOTE: PA2/PA3 pin setup (AF7) must be in HAL_UART_MspInit(). */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================= */
/*                         GPIO                                              */
/* ========================================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Power button / switch: input with pull-up (contact to GND = pressed) */
    GPIO_InitStruct.Pin  = PWR_BTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PWR_BTN_PORT, &GPIO_InitStruct);

    /* HX711 DATA: pull-up so a disconnected HX711 reads "not ready"
     * (timeout) instead of random levels */
    GPIO_InitStruct.Pin  = HX711_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HX711_DATA_GPIO_Port, &GPIO_InitStruct);

    /* HX711 SCK */
    GPIO_InitStruct.Pin   = HX711_SCK_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HX711_SCK_GPIO_Port, &GPIO_InitStruct);

    /* LD2 */
    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /* Set initial output levels AFTER the pins are configured */
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

/* ========================================================================= */
/*                         ERROR HANDLER                                     */
/* ========================================================================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif

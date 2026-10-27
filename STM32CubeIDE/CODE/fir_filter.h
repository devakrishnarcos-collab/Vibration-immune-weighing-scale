/**
 * fir_filter.h - Windowed-sinc FIR low-pass filter (header-only)
 *
 * Project settings: fs = 80 SPS (HX711 RATE pin HIGH), fc = 3 Hz, 63 taps.
 *
 *   static FIR_t f;
 *   FIR_DesignLowpass(&f, 3.0f, 80.0f);
 *   FIR_Prime(&f, first_sample);     // optional, avoids start-up ramp
 *   y = FIR_Update(&f, x);           // once per input sample
 *
 * fc must be below fs/2. Link with -lm and enable the FPU (CubeIDE default).
 */
#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#include <stdint.h>
#include <math.h>

#define FIR_TAPS 63            /* odd -> linear phase, delay = 31 samples */

typedef struct
{
    float    coeff[FIR_TAPS];
    float    buf[FIR_TAPS];    /* circular buffer of past inputs */
    uint16_t idx;
} FIR_t;

/* Hamming-windowed sinc low-pass, normalised to unity DC gain. */
static inline void FIR_DesignLowpass(FIR_t *f, float fc_hz, float fs_hz)
{
    const float wc  = 2.0f * (float)M_PI * fc_hz / fs_hz;   /* rad/sample */
    const float mid = (FIR_TAPS - 1) / 2.0f;
    float sum = 0.0f;

    for (int n = 0; n < FIR_TAPS; n++)
    {
        float m = (float)n - mid;
        float h = (m == 0.0f) ? (wc / (float)M_PI)
                              : (sinf(wc * m) / ((float)M_PI * m));
        float w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * (float)n
                                       / (float)(FIR_TAPS - 1));
        f->coeff[n] = h * w;
        sum += f->coeff[n];
    }

    for (int n = 0; n < FIR_TAPS; n++)
        f->coeff[n] /= sum;

    for (int n = 0; n < FIR_TAPS; n++)
        f->buf[n] = 0.0f;
    f->idx = 0;
}

static inline void FIR_Prime(FIR_t *f, float value)
{
    for (int n = 0; n < FIR_TAPS; n++)
        f->buf[n] = value;
}


static inline float FIR_Update(FIR_t *f, float x)
{
    float acc = 0.0f;

    /* Store the newest sample. */
    f->buf[f->idx] = x;

    /* Apply coefficients to newest-to-oldest samples. */
    uint16_t pos = f->idx;

    for (int k = 0; k < FIR_TAPS; k++)
    {
        acc += f->coeff[k] * f->buf[pos];

        if (pos == 0)
        {
            pos = FIR_TAPS - 1;
        }
        else
        {
            pos--;
        }
    }

    /* Advance circular-buffer index for the next sample. */
    f->idx++;

    if (f->idx >= FIR_TAPS)
    {
        f->idx = 0;
    }

    return acc;
}

#endif /* FIR_FILTER_H */

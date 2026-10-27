/**
 * stable_weight.h - Stable-weight (motion) detector, header-only
 *
 * Keeps the last STABLE_WINDOW filtered net-count samples (80 samples = about
 * 1 s at 80 SPS). The signal is called STABLE when the spread between the
 * highest and the lowest sample in that window (max - min) is within a band.
 *
 *   static Stable_t st;
 *   Stable_Init(&st, band_counts);            // once
 *   float mean;
 *   if (Stable_Update(&st, net_counts, &mean))   // once per sample
 *   {
 *       // stable: "mean" is the average net counts over the last second
 *   }
 *
 * Until the window has filled once (first second) the result is "not stable".
 */
#ifndef STABLE_WEIGHT_H
#define STABLE_WEIGHT_H

#include <stdint.h>

#define STABLE_WINDOW   80        /* samples (about 1 s at 80 SPS) */

typedef struct
{
    float    buf[STABLE_WINDOW];  /* circular buffer of net counts */
    uint16_t idx;
    uint16_t count;               /* samples stored, saturates at window */
    float    band_counts;         /* max allowed (max - min) in the window */
} Stable_t;

static inline void Stable_Init(Stable_t *s, float band_counts)
{
    for (int i = 0; i < STABLE_WINDOW; i++)
        s->buf[i] = 0.0f;

    s->idx         = 0;
    s->count       = 0;
    s->band_counts = band_counts;
}

/* Add one sample. Returns 1 if the signal is stable, else 0.
 * *mean receives the window average (valid when the window is full). */
static inline uint8_t Stable_Update(Stable_t *s, float x, float *mean)
{
    s->buf[s->idx] = x;

    if (++s->idx >= STABLE_WINDOW)
        s->idx = 0;

    if (s->count < STABLE_WINDOW)
        s->count++;

    if (s->count < STABLE_WINDOW)
        return 0;                 /* window not full yet */

    float mn  = s->buf[0];
    float mx  = s->buf[0];
    float sum = 0.0f;

    for (int i = 0; i < STABLE_WINDOW; i++)
    {
        float v = s->buf[i];

        sum += v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }

    *mean = sum / (float)STABLE_WINDOW;

    return ((mx - mn) <= s->band_counts) ? 1 : 0;
}

#endif /* STABLE_WEIGHT_H */

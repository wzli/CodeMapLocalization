#pragma once
#include <stdint.h>
#include <complex.h>

#define SWAP(X, Y)                                                      \
    do {                                                                \
        uint8_t* swap_x_ptr = (uint8_t*) &(X);                          \
        uint8_t* swap_y_ptr = (uint8_t*) &(Y);                          \
        for (uint16_t swap_idx = 0; swap_idx < sizeof(X); ++swap_idx) { \
            uint8_t swap_tmp = swap_x_ptr[swap_idx];                    \
            swap_x_ptr[swap_idx] = swap_y_ptr[swap_idx];                \
            swap_y_ptr[swap_idx] = swap_tmp;                            \
        }                                                               \
    } while (0)

#define QUICK_SELECT(ARRAY, LEN, K)                             \
    for (int32_t start = 0, end = (LEN);;) {                    \
        int32_t pivot = start;                                  \
        for (int32_t index = start; index < end - 1; index++) { \
            if ((ARRAY)[index] <= (ARRAY)[end - 1]) {           \
                SWAP((ARRAY)[index], (ARRAY)[pivot]);           \
                pivot++;                                        \
            }                                                   \
        }                                                       \
        SWAP((ARRAY)[end - 1], (ARRAY)[pivot]);                 \
        if ((K) == pivot) {                                     \
            break;                                              \
        } else if (pivot > (K)) {                               \
            end = pivot;                                        \
        } else {                                                \
            start = pivot;                                      \
        }                                                       \
    }

#define SAMPLE_DATA(ARRAY, INDEX, STRIDE) (ARRAY)[(INDEX) * (STRIDE)]

#define L1_DISTANCE_TRANSFORM_1D(DST, SRC, LEN, STRIDE)                                            \
    do {                                                                                           \
        (DST)[0] = (SRC)[0];                                                                       \
        for (int16_t index = 1; index < (LEN); ++index) {                                          \
            SAMPLE_DATA(DST, index, STRIDE) =                                                      \
                    MIN(SAMPLE_DATA(DST, index - 1, STRIDE) + 1, SAMPLE_DATA(SRC, index, STRIDE)); \
        }                                                                                          \
        for (int16_t index = (LEN) -2; index >= 0; --index) {                                      \
            SAMPLE_DATA(DST, index, STRIDE) =                                                      \
                    MIN(SAMPLE_DATA(DST, index + 1, STRIDE) + 1, SAMPLE_DATA(DST, index, STRIDE)); \
        }                                                                                          \
    } while (0)

#define PARABOLA_INTERSECTION(X2, Y2, X1, Y1) \
    (((X2) - (X1)) * ((X2) + (X1) + 1) + (Y2) - (Y1)) / (2 * ((X2) - (X1)))

#define SQUARE_DISTANCE_TRANSFORM_1D(DST, SRC, LEN, STRIDE)                                     \
    do {                                                                                        \
        SAMPLE_DATA(DST, 0, STRIDE) = 0;                                                        \
        int16_t envelope_index = 0;                                                             \
        int16_t envelope_start = 0;                                                             \
        for (int16_t x = 1; x < (LEN); ++x) {                                                   \
            while (1) {                                                                         \
                int16_t envelope_x = SAMPLE_DATA(DST, envelope_index, STRIDE);                  \
                int16_t intersection = PARABOLA_INTERSECTION(x, SAMPLE_DATA(SRC, x, STRIDE),    \
                        envelope_x, SAMPLE_DATA(SRC, envelope_x, STRIDE));                      \
                if (envelope_start < intersection || envelope_index == 0) {                     \
                    SAMPLE_DATA(DST, ++envelope_index, STRIDE) = x;                             \
                    envelope_start = intersection;                                              \
                    break;                                                                      \
                }                                                                               \
                if (!--envelope_index) {                                                        \
                    envelope_start = 0;                                                         \
                    continue;                                                                   \
                };                                                                              \
                envelope_x = SAMPLE_DATA(DST, envelope_index, STRIDE);                          \
                int16_t prev_envelope_x = SAMPLE_DATA(DST, envelope_index - 1, STRIDE);         \
                envelope_start =                                                                \
                        PARABOLA_INTERSECTION(envelope_x, SAMPLE_DATA(SRC, envelope_x, STRIDE), \
                                prev_envelope_x, SAMPLE_DATA(SRC, prev_envelope_x, STRIDE));    \
            }                                                                                   \
        }                                                                                       \
        envelope_start = CLAMP(envelope_start, 0, (LEN) -1);                                    \
        int16_t envelope_end = (LEN) -1;                                                        \
        for (; envelope_end > envelope_start; --envelope_end) {                                 \
            SAMPLE_DATA(DST, envelope_end, STRIDE) = SAMPLE_DATA(DST, envelope_index, STRIDE);  \
        }                                                                                       \
        while (--envelope_index) {                                                              \
            int16_t envelope_x = SAMPLE_DATA(DST, envelope_index, STRIDE);                      \
            int16_t prev_envelope_x = SAMPLE_DATA(DST, envelope_index - 1, STRIDE);             \
            envelope_start =                                                                    \
                    PARABOLA_INTERSECTION(envelope_x, SAMPLE_DATA(SRC, envelope_x, STRIDE),     \
                            prev_envelope_x, SAMPLE_DATA(SRC, prev_envelope_x, STRIDE));        \
            envelope_start = CLAMP(envelope_start, 0, (LEN) -1);                                \
            for (int16_t x = envelope_start; x < envelope_end; ++x) {                           \
                SAMPLE_DATA(DST, x + 1, STRIDE) = SAMPLE_DATA(DST, envelope_index, STRIDE);     \
            }                                                                                   \
            envelope_end = envelope_start;                                                      \
        }                                                                                       \
        while (envelope_start-- > 0) {                                                          \
            SAMPLE_DATA(DST, envelope_start + 1, STRIDE) = SAMPLE_DATA(DST, 0, STRIDE);         \
        }                                                                                       \
        for (int16_t x = 0; x < (LEN); ++x) {                                                   \
            int16_t envelope_x = SAMPLE_DATA(DST, MIN(x, (LEN) -1), STRIDE);                    \
            int16_t dx = x - envelope_x;                                                        \
            SAMPLE_DATA(DST, x, STRIDE) = SAMPLE_DATA(SRC, envelope_x, STRIDE) + SQR(dx);       \
        }                                                                                       \
    } while (0)

#define BIT_REVERSE_PERMUTATION(ARRAY, LEN)                   \
    for (int index = 0, target = 0; index < (LEN); ++index) { \
        if (target > index) {                                 \
            SWAP((ARRAY)[index], (ARRAY)[target]);            \
        }                                                     \
        int mask = (LEN);                                     \
        while (target & (mask >>= 1)) {                       \
            target &= ~mask;                                  \
        }                                                     \
        target |= mask;                                       \
    }

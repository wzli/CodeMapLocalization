#pragma once
#include "math_utils.h"
#include "generic_algorithms.h"

typedef struct {
    int16_t x;
    int16_t y;
} ImagePoint;

#define IMG_MATRIX_TYPEDEF(NAME, TYPE) \
    typedef struct {                   \
        TYPE* data;                    \
        ImagePoint size;               \
    } NAME

IMG_MATRIX_TYPEDEF(ImageMatrix, uint8_t);
IMG_MATRIX_TYPEDEF(ImageMatrixInt8, int8_t);
IMG_MATRIX_TYPEDEF(ImageMatrixInt32, int32_t);
IMG_MATRIX_TYPEDEF(ImageMatrixComplex, float complex);

// type generic macro functions for image manipulation

#define PIXEL(MAT, ROW, COL) (MAT).data[(ROW) * (MAT).size.x + (COL)]

#define FOR_EACH_NESTED_PIXEL(MAT, LEVEL)                                 \
    for (int16_t LEVEL##row = 0; LEVEL##row < (MAT).size.y; ++LEVEL##row) \
        for (int16_t LEVEL##col = 0; LEVEL##col < (MAT).size.x; ++LEVEL##col)

#define FOR_EACH_PIXEL(MAT) FOR_EACH_NESTED_PIXEL(MAT, )

#define IMG_CHECK_BOUNDS(MAT, ROW, COL, OFFSET)                                   \
    ((ROW) >= (OFFSET) && (COL) >= (OFFSET) && (ROW) + (OFFSET) < (MAT).size.y && \
            (COL) + (OFFSET) < (MAT).size.x)

#define IMG_SIZE(MAT) ((MAT).size.y * (MAT).size.x)

#define IMG_SET_SIZE(DST, N_COLS, N_ROWS) ((DST).size.x = (N_COLS), (DST).size.y = (N_ROWS))

#define IMG_COPY(DST, SRC)                                                   \
    do {                                                                     \
        (DST).size = (SRC).size;                                             \
        FOR_EACH_PIXEL(SRC) { PIXEL(DST, row, col) = PIXEL(SRC, row, col); } \
    } while (0)

#define IMG_FILL(MAT, VAL) FOR_EACH_PIXEL(MAT)(PIXEL(MAT, row, col) = (VAL))

#define IMG_PIXEL_SUM(SUM, MAT) FOR_EACH_PIXEL(MAT)(SUM += PIXEL(MAT, row, col))

#define IMG_PIXEL_MAX(MAX_VAL, MAT) \
    FOR_EACH_PIXEL(MAT)(MAX_VAL = MAX(MAX_VAL, PIXEL(MAT, row, col)))

#define IMG_PIXEL_MIN(MIN_VAL, MAT) \
    FOR_EACH_PIXEL(MAT)(MIN_VAL = MIN(MIN_VAL, PIXEL(MAT, row, col)))

#define IMG_PIXEL_AVERAGE(AVG, MAT) \
    do {                            \
        IMG_PIXEL_SUM(AVG, MAT);    \
        AVG /= IMG_SIZE(MAT);       \
    } while (0)

#define IMG_PIXEL_WEIGHTED_SUM(SUM, KERNEL, MAT, ROW, COL) \
    FOR_EACH_NESTED_PIXEL(KERNEL, k_)                      \
    ((SUM) += PIXEL(KERNEL, k_row, k_col) * PIXEL(MAT, k_row + (ROW), k_col + (COL)))

#define IMG_VALID_PADDING(DST, SRC, KERNEL) \
    IMG_SET_SIZE(DST, (SRC).size.x - (KERNEL).size.x + 1, (SRC).size.y - (KERNEL).size.y + 1)

#define IMG_CONVOLUTION(DST, SRC, KERNEL, SCALE, CLAMP_MIN, CLAMP_MAX)         \
    do {                                                                       \
        IMG_VALID_PADDING(DST, SRC, KERNEL);                                   \
        FOR_EACH_PIXEL(DST) {                                                  \
            int32_t value = 0;                                                 \
            IMG_PIXEL_WEIGHTED_SUM(value, KERNEL, SRC, row, col);              \
            PIXEL(DST, row, col) = CLAMP(value * SCALE, CLAMP_MIN, CLAMP_MAX); \
        }                                                                      \
    } while (0)

#define IMG_REDUCE_FILTER(DST, SRC, BLOCK_SIZE, REDUCE_FUNC)                             \
    do {                                                                                 \
        (DST).size = (SRC).size;                                                         \
        (DST).size.x -= (BLOCK_SIZE) -1;                                                 \
        FOR_EACH_PIXEL(DST) {                                                            \
            PIXEL(DST, row, col) = PIXEL(SRC, row, col);                                 \
            for (int16_t index = 1; index < (BLOCK_SIZE); ++index) {                     \
                PIXEL(DST, row, col) =                                                   \
                        REDUCE_FUNC(PIXEL(DST, row, col), PIXEL(SRC, row, col + index)); \
            }                                                                            \
        }                                                                                \
        (DST).size.y -= (BLOCK_SIZE) -1;                                                 \
        FOR_EACH_PIXEL(DST) {                                                            \
            for (int16_t index = 1; index < (BLOCK_SIZE); ++index) {                     \
                PIXEL(DST, row, col) =                                                   \
                        REDUCE_FUNC(PIXEL(DST, row, col), PIXEL(DST, row + index, col)); \
            }                                                                            \
        }                                                                                \
    } while (0)

#define IMG_THRESHOLD(DST, SRC, THRESH)                                           \
    do {                                                                          \
        (DST).size = (SRC).size;                                                  \
        FOR_EACH_PIXEL(SRC) {                                                     \
            PIXEL(DST, row, col) = (PIXEL(SRC, row, col) > (THRESH)) * UINT8_MAX; \
        }                                                                         \
    } while (0)

#define IMG_CROP(DST, SRC, TOP_LEFT)                                                             \
    if (IMG_CHECK_BOUNDS(SRC, (TOP_LEFT).y, (TOP_LEFT).x, 0) &&                                  \
            IMG_CHECK_BOUNDS(                                                                    \
                    SRC, (TOP_LEFT).y + (DST).size.y - 1, (TOP_LEFT).x + (DST).size.x - 1, 0)) { \
        FOR_EACH_PIXEL(DST) {                                                                    \
            PIXEL(DST, row, col) = PIXEL(SRC, row + (TOP_LEFT).y, col + (TOP_LEFT).x);           \
        }                                                                                        \
    } else                                                                                       \
        IMG_SET_SIZE(DST, 0, 0)

#define IMG_FLIP_INPLACE(MAT, F_ROW, F_COL)                   \
    FOR_EACH_PIXEL(MAT)                                       \
    if (row < (F_ROW) || col < (F_COL)) {                     \
        SWAP(PIXEL(MAT, row, col), PIXEL(MAT, F_ROW, F_COL)); \
    } else                                                    \
        break

#define IMG_FLIP(DST, SRC, F_ROW, F_COL)                                         \
    if (&(DST) != &(SRC)) {                                                      \
        (DST).size = (SRC).size;                                                 \
        FOR_EACH_PIXEL(SRC) { PIXEL(DST, row, col) = PIXEL(SRC, F_ROW, F_COL); } \
    } else                                                                       \
        IMG_FLIP_INPLACE(SRC, F_ROW, F_COL)

#define IMG_VFLIP(DST, SRC) IMG_FLIP(DST, SRC, (SRC).size.y - 1 - row, col)
#define IMG_HFLIP(DST, SRC) IMG_FLIP(DST, SRC, row, (SRC).size.x - 1 - col)

#define IMG_TRANSPOSE(DST, SRC)         \
    if ((SRC).size.y == (SRC).size.x) { \
        IMG_FLIP(DST, SRC, col, row);   \
    } else                              \
        IMG_SET_SIZE(DST, 0, 0)

#define IMG_NORMALIZE_RANGE(DST, SRC, MIN, MAX)                                           \
    do {                                                                                  \
        (DST).size = (SRC).size;                                                          \
        FOR_EACH_PIXEL(SRC) {                                                             \
            int32_t val = ((PIXEL(SRC, row, col) - (MIN)) * UINT8_MAX) / ((MAX) - (MIN)); \
            PIXEL(DST, row, col) = CLAMP(val, 0, UINT8_MAX);                              \
        }                                                                                 \
    } while (0)

#define IMG_NORMALIZE(DST, SRC)                                  \
    do {                                                         \
        int32_t min_pixel = PIXEL(SRC, 0, 0);                    \
        int32_t max_pixel = PIXEL(SRC, 0, 0);                    \
        IMG_PIXEL_MIN(min_pixel, SRC);                           \
        IMG_PIXEL_MAX(max_pixel, SRC);                           \
        if (max_pixel == min_pixel) {                            \
            (DST).size = (SRC).size;                             \
            IMG_FILL(DST, 0);                                    \
        } else {                                                 \
            IMG_NORMALIZE_RANGE(DST, SRC, min_pixel, max_pixel); \
        }                                                        \
    } while (0)

#define IMG_SEPARABLE_2D_TRANSFORM(DST, SRC, TRANSFORM_1D, LINE_BUFFER)                           \
    do {                                                                                          \
        (DST).size = (SRC).size;                                                                  \
        (DST).data += (SRC).size.x * (LINE_BUFFER);                                               \
        for (int16_t col = 0; col < (SRC).size.x; ++col) {                                        \
            TRANSFORM_1D((DST).data + col, (SRC).data + col, (SRC).size.y, (SRC).size.x);         \
        }                                                                                         \
        for (int16_t row = 0; row < (SRC).size.y; ++row, (DST).data += (SRC).size.x) {            \
            TRANSFORM_1D((DST).data - (SRC).size.x * (LINE_BUFFER), (DST).data, (SRC).size.x, 1); \
        }                                                                                         \
        (DST).data -= IMG_SIZE(DST) + (SRC).size.x * (LINE_BUFFER);                               \
    } while (0)

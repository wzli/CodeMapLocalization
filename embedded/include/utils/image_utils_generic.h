#pragma once
#include "math_utils.h"

typedef struct {
    int16_t x;
    int16_t y;
} ImagePoint;

#define IMG_MATRIX_TYPEDEF(NAME, TYPE) \
    typedef struct {                   \
        TYPE* data;                    \
        int16_t n_cols;                \
        int16_t n_rows;                \
    } NAME

IMG_MATRIX_TYPEDEF(ImageMatrix, uint8_t);
IMG_MATRIX_TYPEDEF(ImageMatrixInt8, int8_t);
IMG_MATRIX_TYPEDEF(ImageMatrixInt32, int32_t);

// type generic macro functions for image manipulation

#define PIXEL(MAT, ROW, COL) (MAT).data[(ROW) * (MAT).n_cols + (COL)]

#define FOR_EACH_NESTED_PIXEL(MAT, LEVEL)                                 \
    for (int16_t LEVEL##row = 0; LEVEL##row < (MAT).n_rows; ++LEVEL##row) \
        for (int16_t LEVEL##col = 0; LEVEL##col < (MAT).n_cols; ++LEVEL##col)

#define FOR_EACH_PIXEL(MAT) FOR_EACH_NESTED_PIXEL(MAT, )

#define IMG_CHECK_BOUNDS(MAT, ROW, COL, OFFSET)                                   \
    ((ROW) >= (OFFSET) && (COL) >= (OFFSET) && (ROW) + (OFFSET) < (MAT).n_rows && \
            (COL) + (OFFSET) < (MAT).n_cols)

#define IMG_SIZE(MAT) ((MAT).n_rows * (MAT).n_cols)

#define IMG_SET_SIZE(DST, N_COLS, N_ROWS) ((DST).n_cols = (N_COLS), (DST).n_rows = (N_ROWS))

#define IMG_COPY_SIZE(DST, SRC) IMG_SET_SIZE(DST, (SRC).n_cols, (SRC).n_rows)

#define IMG_COPY(DST, SRC)                                                   \
    do {                                                                     \
        IMG_COPY_SIZE(DST, SRC);                                             \
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

#define IMG_APPLY_KERNEL(SUM, KERNEL, MAT, ROW, COL) \
    FOR_EACH_NESTED_PIXEL(KERNEL, k_)                \
    ((SUM) += PIXEL(KERNEL, k_row, k_col) * PIXEL(MAT, k_row + (ROW), k_col + (COL)))

#define IMG_VALID_PADDING(DST, SRC, KERNEL) \
    IMG_SET_SIZE(DST, (SRC).n_cols - (KERNEL).n_cols + 1, (SRC).n_rows - (KERNEL).n_rows + 1)

#define IMG_CONVOLUTION(DST, SRC, KERNEL, SCALE, CLAMP_MIN, CLAMP_MAX)         \
    do {                                                                       \
        IMG_VALID_PADDING(DST, SRC, KERNEL);                                   \
        FOR_EACH_PIXEL(DST) {                                                  \
            int32_t value = 0;                                                 \
            IMG_APPLY_KERNEL(value, KERNEL, SRC, row, col);                    \
            PIXEL(DST, row, col) = CLAMP(value * SCALE, CLAMP_MIN, CLAMP_MAX); \
        }                                                                      \
    } while (0)

#define IMG_REDUCE_FILTER(DST, SRC, KERNEL, INIT_VAL, REDUCE_FUNC)                       \
    do {                                                                                 \
        IMG_VALID_PADDING(DST, SRC, KERNEL);                                             \
        FOR_EACH_PIXEL(DST) {                                                            \
            PIXEL(DST, row, col) = (INIT_VAL);                                           \
            FOR_EACH_NESTED_PIXEL(KERNEL, k_) {                                          \
                if (PIXEL(KERNEL, k_row, k_col)) {                                       \
                    PIXEL(DST, row, col) = REDUCE_FUNC(                                  \
                            PIXEL(DST, row, col), PIXEL(SRC, row + k_row, col + k_col)); \
                }                                                                        \
            }                                                                            \
        }                                                                                \
    } while (0)

#define IMG_THRESHOLD(DST, SRC, THRESH)                                           \
    do {                                                                          \
        IMG_COPY_SIZE(DST, SRC);                                                  \
        FOR_EACH_PIXEL(SRC) {                                                     \
            PIXEL(DST, row, col) = (PIXEL(SRC, row, col) > (THRESH)) * UINT8_MAX; \
        }                                                                         \
    } while (0)

#define IMG_CROP(DST, SRC, TOP_LEFT)                                                             \
    if (IMG_CHECK_BOUNDS(SRC, (TOP_LEFT).y, (TOP_LEFT).x, 0) &&                                  \
            IMG_CHECK_BOUNDS(                                                                    \
                    SRC, (TOP_LEFT).y + (DST).n_rows - 1, (TOP_LEFT).x + (DST).n_cols - 1, 0)) { \
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
        IMG_COPY_SIZE(DST, SRC);                                                 \
        FOR_EACH_PIXEL(SRC) { PIXEL(DST, row, col) = PIXEL(SRC, F_ROW, F_COL); } \
    } else                                                                       \
        IMG_FLIP_INPLACE(SRC, F_ROW, F_COL)

#define IMG_VFLIP(DST, SRC) IMG_FLIP(DST, SRC, (SRC).n_rows - 1 - row, col)
#define IMG_HFLIP(DST, SRC) IMG_FLIP(DST, SRC, row, (SRC).n_cols - 1 - col)

#define IMG_TRANSPOSE(DST, SRC)         \
    if ((SRC).n_rows == (SRC).n_cols) { \
        IMG_FLIP(DST, SRC, col, row);   \
    } else                              \
        IMG_SET_SIZE(DST, 0, 0)

#define IMG_NORMALIZE_RANGE(DST, SRC, MIN, MAX)                                           \
    do {                                                                                  \
        IMG_COPY_SIZE(DST, SRC);                                                          \
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
            IMG_COPY_SIZE(DST, SRC);                             \
            IMG_FILL(DST, 0);                                    \
        } else {                                                 \
            IMG_NORMALIZE_RANGE(DST, SRC, min_pixel, max_pixel); \
        }                                                        \
    } while (0)

#define CH_DATA(ARRAY, INDEX, CHANNEL) (ARRAY)[(INDEX) * (CHANNEL)]

#define L1_DISTANCE_TRANSFORM_1D(DST, SRC, LEN, STRIDE)                            \
    do {                                                                           \
        (DST)[0] = (SRC)[0];                                                       \
        for (int16_t I = 1; I < (LEN); ++I) {                                      \
            CH_DATA(DST, I, STRIDE) =                                              \
                    MIN(CH_DATA(DST, I - 1, STRIDE) + 1, CH_DATA(SRC, I, STRIDE)); \
        }                                                                          \
        for (int16_t I = (LEN) -2; I >= 0; --I) {                                  \
            CH_DATA(DST, I, STRIDE) =                                              \
                    MIN(CH_DATA(DST, I + 1, STRIDE) + 1, CH_DATA(DST, I, STRIDE)); \
        }                                                                          \
    } while (0)

#define SQUARE_DISTANCE_TRANSFORM_1D(DST, SRC, LEN, STRIDE)                                       \
    do {                                                                                          \
        for (int16_t x = 0; x < (LEN); ++x) {                                                     \
            CH_DATA(DST, x, STRIDE) = 0;                                                          \
        }                                                                                         \
        for (int16_t x = 1; x < (LEN); ++x) {                                                     \
            CH_DATA(DST, x, STRIDE) = MAX(CH_DATA(DST, x, STRIDE), CH_DATA(DST, x - 1, STRIDE));  \
            int16_t dx = x - CH_DATA(DST, x, STRIDE);                                             \
            int16_t dy = CH_DATA(SRC, x, STRIDE) - CH_DATA(SRC, CH_DATA(DST, x, STRIDE), STRIDE); \
            int16_t s = (dy + dx * (x + CH_DATA(DST, x, STRIDE) + 1)) / (2 * dx);                 \
            if (s < x) {                                                                          \
                CH_DATA(DST, x, STRIDE) = x;                                                      \
            } else if (s < (LEN)) {                                                               \
                CH_DATA(DST, s, STRIDE) = x;                                                      \
            }                                                                                     \
        }                                                                                         \
        for (int16_t x = (LEN) -1, min_x = CH_DATA(DST, (LEN) -1, STRIDE); x >= 0; --x) {         \
            int16_t dx = x - CH_DATA(DST, x, STRIDE);                                             \
            int16_t y = SQR(dx) + CH_DATA(SRC, CH_DATA(DST, x, STRIDE), STRIDE);                  \
            int16_t min_dx = x - min_x;                                                           \
            int16_t min_y = SQR(min_dx) + CH_DATA(SRC, min_x, STRIDE);                            \
            if (y <= min_y) {                                                                     \
                min_x = CH_DATA(DST, x, STRIDE);                                                  \
                CH_DATA(DST, x, STRIDE) = y;                                                      \
            } else {                                                                              \
                CH_DATA(DST, x, STRIDE) = min_y;                                                  \
            }                                                                                     \
        }                                                                                         \
    } while (0)

#define IMG_SEPARABLE_2D_TRANSFORM(DST, SRC, TRANSFORM_1D, LINE_BUFFER)                           \
    do {                                                                                          \
        IMG_COPY_SIZE(DST, SRC);                                                                  \
        (DST).data += (SRC).n_cols * (LINE_BUFFER);                                               \
        for (int16_t col = 0; col < (SRC).n_cols; ++col) {                                        \
            TRANSFORM_1D((DST).data + col, (SRC).data + col, (SRC).n_rows, (SRC).n_cols);         \
        }                                                                                         \
        for (int16_t row = 0; row < (SRC).n_rows; ++row, (DST).data += (SRC).n_cols) {            \
            TRANSFORM_1D((DST).data - (SRC).n_cols * (LINE_BUFFER), (DST).data, (SRC).n_cols, 1); \
        }                                                                                         \
        (DST).data -= IMG_SIZE(DST) + (SRC).n_cols * (LINE_BUFFER);                               \
    } while (0)

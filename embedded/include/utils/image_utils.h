#pragma once
#include "math_utils.h"

#define IMG_MATRIX_TYPEDEF(NAME, TYPE) \
    typedef struct {                   \
        TYPE* data;                    \
        int16_t n_cols;                \
        int16_t n_rows;                \
    } NAME

IMG_MATRIX_TYPEDEF(ImageMatrix, uint8_t);
IMG_MATRIX_TYPEDEF(ImageMatrixInt8, int8_t);
IMG_MATRIX_TYPEDEF(ImageMatrixInt32, int32_t);

typedef uint8_t (*ImageInterpolation)(const ImageMatrix mat, Vector2f position);

typedef struct {
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;
} ImageWindow;

static const ImageMatrixInt8 edge_kernel = {(int8_t[]){-1, -1, -1, -1, 8, -1, -1, -1, -1}, 3, 3};
static const ImageMatrixInt8 sobel_x_kernel = {(int8_t[]){-1, 0, 1, -2, 0, 2, -1, 0, 1}, 3, 3};
static const ImageMatrixInt8 sobel_y_kernel = {(int8_t[]){-1, -2, -1, 0, 0, 0, 1, 2, 1}, 3, 3};

#define PIXEL(MATRIX, ROW, COL) ((MATRIX).data[(ROW) * (MATRIX).n_cols + (COL)])

#define FOR_EACH_PIXEL(MAT)                          \
    for (int16_t row = 0; row < (MAT).n_rows; ++row) \
        for (int16_t col = 0; col < (MAT).n_cols; ++col)

#define IMG_SIZE(MAT) ((MAT).n_rows * (MAT).n_cols)

#define IMG_COPY_SIZE(DST_PTR, SRC)       \
    do {                                  \
        (DST_PTR)->n_rows = (SRC).n_rows; \
        (DST_PTR)->n_cols = (SRC).n_cols; \
    } while (0)

#define IMG_COPY(DST_PTR, SRC)                                                    \
    do {                                                                          \
        IMG_COPY_SIZE(DST_PTR, SRC);                                              \
        FOR_EACH_PIXEL(SRC) { PIXEL(*DST_PTR, row, col) = PIXEL(SRC, row, col); } \
    } while (0)

#define IMG_FILL(MAT, VAL) \
    FOR_EACH_PIXEL(MAT) { PIXEL(MAT, row, col) = (VAL); }

#define IMG_SUM(SUM, MAT) \
    FOR_EACH_PIXEL(MAT) { SUM += PIXEL(MAT, row, col); }

#define IMG_MAX(MAX_VAL, MAT) \
    FOR_EACH_PIXEL(MAT) { MAX_VAL = MAX(MAX_VAL, PIXEL(MAT, row, col)); }

#define IMG_MIN(MIN_VAL, MAT) \
    FOR_EACH_PIXEL(MAT) { MIN_VAL = MIN(MIN_VAL, PIXEL(MAT, row, col)); }

#define IMG_AVERAGE(AVG, MAT) \
    do {                      \
        IMG_SUM(AVG, MAT);    \
        AVG /= IMG_SIZE(MAT); \
    } while (0)

#define IMG_APPLY_KERNEL(ACCUMULATOR, KERNEL, MAT, ROW, COL)                                \
    for (int16_t k_row = 0; k_row < (KERNEL).n_rows; ++k_row)                               \
        for (int16_t k_col = 0; k_col < (KERNEL).n_cols; ++k_col) {                         \
            (ACCUMULATOR) +=                                                                \
                    PIXEL(KERNEL, k_row, k_col) * PIXEL(MAT, k_row + (ROW), k_col + (COL)); \
        }

#define IMG_THRESHOLD(DST_PTR, SRC, THRESH)                                            \
    do {                                                                               \
        IMG_COPY_SIZE(DST_PTR, SRC);                                                   \
        FOR_EACH_PIXEL(SRC) {                                                          \
            PIXEL(*DST_PTR, row, col) = (PIXEL(SRC, row, col) > (THRESH)) * UINT8_MAX; \
        }                                                                              \
    } while (0)

#define IMG_CROP(DST_PTR, SRC, WIN)                                                 \
    do {                                                                            \
        (DST_PTR)->n_cols = (WIN).x1 - (WIN).x0;                                    \
        (DST_PTR)->n_rows = (WIN).y1 - (WIN).y0;                                    \
        FOR_EACH_PIXEL(*DST_PTR) {                                                  \
            PIXEL(*DST_PTR, row, col) = PIXEL(SRC, row + (WIN).y0, col + (WIN).x0); \
        }                                                                           \
    } while (0)

#define IMG_VFLIP_INPLACE(MAT)                                    \
    FOR_EACH_PIXEL(MAT) {                                         \
        int16_t flipped_row = (MAT).n_rows - row - 1;             \
        if (row >= flipped_row) {                                 \
            break;                                                \
        }                                                         \
        SWAP(PIXEL(MAT, row, col), PIXEL(MAT, flipped_row, col)); \
    }

#define IMG_HFLIP_INPLACE(MAT)                                    \
    FOR_EACH_PIXEL(MAT) {                                         \
        int16_t flipped_col = (MAT).n_cols - col - 1;             \
        if (col >= flipped_col) {                                 \
            break;                                                \
        }                                                         \
        SWAP(PIXEL(MAT, row, col), PIXEL(MAT, row, flipped_col)); \
    }

#define IMG_VFLIP(DST_PTR, SRC)                                                      \
    do {                                                                             \
        if ((DST_PTR) == &(SRC)) {                                                   \
            IMG_VFLIP_INPLACE(SRC);                                                  \
        } else {                                                                     \
            IMG_COPY_SIZE(DST_PTR, SRC);                                             \
            FOR_EACH_PIXEL(SRC) {                                                    \
                PIXEL(*DST_PTR, row, col) = PIXEL(SRC, (SRC).n_rows - row - 1, col); \
            }                                                                        \
        }                                                                            \
    } while (0)

#define IMG_HFLIP(DST_PTR, SRC)                                                      \
    do {                                                                             \
        if ((DST_PTR) == &(SRC)) {                                                   \
            IMG_HFLIP_INPLACE(SRC);                                                  \
        } else {                                                                     \
            IMG_COPY_SIZE(DST_PTR, SRC);                                             \
            FOR_EACH_PIXEL(SRC) {                                                    \
                PIXEL(*DST_PTR, row, col) = PIXEL(SRC, row, (SRC).n_cols - col - 1); \
            }                                                                        \
        }                                                                            \
    } while (0)

#define IMG_NORMALIZE_RANGE(DST_PTR, SRC, MIN, MAX)                                       \
    do {                                                                                  \
        IMG_COPY_SIZE(DST_PTR, SRC);                                                      \
        FOR_EACH_PIXEL(SRC) {                                                             \
            int32_t val = ((PIXEL(SRC, row, col) - (MIN)) * UINT8_MAX) / ((MAX) - (MIN)); \
            PIXEL(*DST_PTR, row, col) = CLAMP(val, 0, UINT8_MAX);                         \
        }                                                                                 \
    } while (0)

#define IMG_NORMALIZE(DST_PTR, SRC)                                  \
    do {                                                             \
        int32_t min_pixel = PIXEL(SRC, 0, 0);                        \
        int32_t max_pixel = PIXEL(SRC, 0, 0);                        \
        IMG_MIN(min_pixel, SRC);                                     \
        IMG_MAX(max_pixel, SRC);                                     \
        if (max_pixel == min_pixel) {                                \
            IMG_COPY_SIZE(DST_PTR, SRC);                             \
            IMG_FILL(*DST_PTR, 0);                                   \
        } else {                                                     \
            IMG_NORMALIZE_RANGE(DST_PTR, SRC, min_pixel, max_pixel); \
        }                                                            \
    } while (0)

uint8_t img_nearest_interpolation(const ImageMatrix mat, Vector2f position);
uint8_t img_bilinear_interpolation(const ImageMatrix mat, Vector2f position);
uint8_t img_bicubic_interpolation(const ImageMatrix mat, Vector2f position);

void img_resize(ImageMatrix dst, const ImageMatrix src, ImageInterpolation interpolation);
void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, uint8_t bg_fill,
        ImageInterpolation interpolation);

void img_histogram(uint32_t* histogram, const ImageMatrix mat);
uint8_t img_otsu_histogram_threshold(const uint32_t* histogram);

void img_draw_line(ImageMatrix mat, ImageWindow line, uint8_t color);

void img_convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrixInt8 kernel);
void img_edge_filter(ImageMatrix* dst, const ImageMatrix src);

void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src);

void img_hough_line_transform(ImageMatrixInt32 dst, const ImageMatrix src);

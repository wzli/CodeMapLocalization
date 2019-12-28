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
IMG_MATRIX_TYPEDEF(ImageMatrixFloat, float);
typedef uint8_t (*ImageInterpolation)(const ImageMatrix mat, Vector2f position);

static const ImageMatrixInt8 edge_kernel = {(int8_t[]){-1, -1, -1, -1, 8, -1, -1, -1, -1}, 3, 3};
static const ImageMatrixInt8 sobel_x_kernel = {(int8_t[]){-1, 0, 1, -2, 0, 2, -1, 0, 1}, 3, 3};
static const ImageMatrixInt8 sobel_y_kernel = {(int8_t[]){-1, -2, -1, 0, 0, 0, 1, 2, 1}, 3, 3};

#define PIXEL(MATRIX, ROW, COL) ((MATRIX).data[(ROW) * (MATRIX).n_cols + (COL)])

#define FOR_EACH_PIXEL_PREFIXED_INDEX(MAT, PRE)                     \
    for (int16_t PRE##row = 0; PRE##row < (MAT).n_rows; ++PRE##row) \
        for (int16_t PRE##col = 0; PRE##col < (MAT).n_cols; ++PRE##col)

#define FOR_EACH_PIXEL(MAT) FOR_EACH_PIXEL_PREFIXED_INDEX(MAT, )

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

#define IMG_APPLY_KERNEL(ACCUMULATOR, KERNEL, MAT, ROW, COL)                                     \
    FOR_EACH_PIXEL_PREFIXED_INDEX(KERNEL, k_) {                                                  \
        (ACCUMULATOR) += PIXEL(KERNEL, k_row, k_col) * PIXEL(MAT, k_row + (ROW), k_col + (COL)); \
    }

uint8_t img_average(const ImageMatrix mat);
void img_threshold(ImageMatrix* dst, const ImageMatrix src, uint8_t threshold);
void img_normalize(ImageMatrix* dst, const ImageMatrix src);

void img_draw_line(ImageMatrix mat, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);

uint8_t img_bilinear_interpolation(const ImageMatrix mat, Vector2f position);
void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, uint8_t bg_fill,
        ImageInterpolation interpolation);

void img_convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrixInt8 kernel);
void img_edge_filter(ImageMatrix* dst, const ImageMatrix src);

void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src);

uint32_t img_count_negative(ImageMatrixFloat mat);
void img_hough_line_transform(ImageMatrixFloat dst, const ImageMatrix src);

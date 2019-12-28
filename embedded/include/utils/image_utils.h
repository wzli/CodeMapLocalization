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

#define FOR_EACH_PIXEL_PREFIXED_INDEX(MAT, PRE)                     \
    for (int16_t PRE##row = 0; PRE##row < (MAT).n_rows; ++PRE##row) \
        for (int16_t PRE##col = 0; PRE##col < (MAT).n_cols; ++PRE##col)

#define FOR_EACH_PIXEL(MAT) FOR_EACH_PIXEL_PREFIXED_INDEX(MAT, )

#define FOR_EACH_PIXEL_IN_WINDOW(WIN)                   \
    for (int16_t row = (WIN).x0; row < (WIN).x1; ++row) \
        for (int16_t col = (WIN).y0; col < (WIN).y1; ++col)

#define IMG_SIZE(MAT) ((MAT).n_rows * (MAT).n_cols)

#define IMG_SIZE_WINDOW(WIN) (((WIN).x1 - (WIN).x0) * ((WIN).y1 - (WIN).y0))

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

#define IMG_SUM(SUM, MAT) \
    FOR_EACH_PIXEL(MAT) { SUM += PIXEL(MAT, row, col); }

#define IMG_SUM_WINDOW(SUM, MAT, WIN) \
    FOR_EACH_PIXEL_IN_WINDOW(WIN) { SUM += PIXEL(MAT, row, col); }

#define IMG_THRESHOLD(DST_PTR, SRC, THRESH)                                             \
    do {                                                                                \
        IMG_COPY_SIZE(DST_PTR, SRC);                                                    \
        FOR_EACH_PIXEL(SRC) {                                                           \
            PIXEL(*DST_PTR, row, col) = (PIXEL(SRC, row, col) >= (THRESH)) * UINT8_MAX; \
        }                                                                               \
    } while (0)

#define IMG_CROP(DST_PTR, SRC, WIN)                                                        \
    do {                                                                                   \
        (DST_PTR)->n_cols = (WIN).x1 - (WIN).x0;                                           \
        (DST_PTR)->n_rows = (WIN).y1 - (WIN).y0;                                           \
        FOR_EACH_PIXEL_IN_WINDOW(WIN) { PIXEL(*DST_PTR, row, col) = PIXEL(SRC, row, col) } \
    } while (0)

#define IMG_NORMALIZE(DST_PTR, SRC)                                                            \
    do {                                                                                       \
        int32_t max_pixel = PIXEL(SRC, 0, 0);                                                  \
        int32_t min_pixel = PIXEL(SRC, 0, 0);                                                  \
        FOR_EACH_PIXEL(SRC) {                                                                  \
            max_pixel = MAX(max_pixel, PIXEL(SRC, row, col));                                  \
            min_pixel = MIN(min_pixel, PIXEL(SRC, row, col));                                  \
        }                                                                                      \
        IMG_COPY_SIZE(DST_PTR, SRC);                                                           \
        if (max_pixel == min_pixel) {                                                          \
            IMG_FILL(*DST_PTR, 0);                                                             \
        } else {                                                                               \
            FOR_EACH_PIXEL(SRC) {                                                              \
                PIXEL(*DST_PTR, row, col) = ((PIXEL(SRC, row, col) - min_pixel) * UINT8_MAX) / \
                                            (max_pixel - min_pixel);                           \
            }                                                                                  \
        }                                                                                      \
    } while (0)

uint8_t img_bilinear_interpolation(const ImageMatrix mat, Vector2f position);

void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, uint8_t bg_fill,
        ImageInterpolation interpolation);

void img_draw_line(ImageMatrix mat, ImageWindow line, uint8_t color);

void img_convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrixInt8 kernel);

void img_edge_filter(ImageMatrix* dst, const ImageMatrix src);

void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src);

void img_hough_line_transform(ImageMatrixInt32 dst, const ImageMatrix src);

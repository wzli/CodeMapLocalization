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
    int16_t x;
    int16_t y;
} ImagePoint;

// type generic macro functions for image manipulation

#define PIXEL(MATRIX, ROW, COL) (MATRIX).data[(ROW) * (MATRIX).n_cols + (COL)]

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

#define IMG_SUM(SUM, MAT) FOR_EACH_PIXEL(MAT)(SUM += PIXEL(MAT, row, col))

#define IMG_MAX(MAX_VAL, MAT) FOR_EACH_PIXEL(MAT)(MAX_VAL = MAX(MAX_VAL, PIXEL(MAT, row, col)))

#define IMG_MIN(MIN_VAL, MAT) FOR_EACH_PIXEL(MAT)(MIN_VAL = MIN(MIN_VAL, PIXEL(MAT, row, col)))

#define IMG_AVERAGE(AVG, MAT) \
    do {                      \
        IMG_SUM(AVG, MAT);    \
        AVG /= IMG_SIZE(MAT); \
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
        IMG_MIN(min_pixel, SRC);                                 \
        IMG_MAX(max_pixel, SRC);                                 \
        if (max_pixel == min_pixel) {                            \
            IMG_COPY_SIZE(DST, SRC);                             \
            IMG_FILL(DST, 0);                                    \
        } else {                                                 \
            IMG_NORMALIZE_RANGE(DST, SRC, min_pixel, max_pixel); \
        }                                                        \
    } while (0)

// convolution kernels
static const ImageMatrixInt8 img_sharpen_kernel = {
        (int8_t[]){-1, -1, -1, -1, 9, -1, -1, -1, -1}, 3, 3};
static const ImageMatrixInt8 img_edge_detect_kernel = {
        (int8_t[]){-1, -1, -1, -1, 8, -1, -1, -1, -1}, 3, 3};
static const ImageMatrixInt8 img_sobel_x_kernel = {(int8_t[]){-1, 0, 1, -2, 0, 2, -1, 0, 1}, 3, 3};
static const ImageMatrixInt8 img_sobel_y_kernel = {(int8_t[]){-1, -2, -1, 0, 0, 0, 1, 2, 1}, 3, 3};
static const ImageMatrixInt8 img_laplacian_kernel = {(int8_t[]){0, 1, 0, 1, -4, 1, 0, 1, 0}, 3, 3};

// shape min/max aka dilate/erode kernels
static const ImageMatrix img_box_2x2_kernel = {(uint8_t[]){1, 1, 1, 1}, 2, 2};
static const ImageMatrix img_box_3x3_kernel = {(uint8_t[]){1, 1, 1, 1, 1, 1, 1, 1, 1}, 3, 3};
static const ImageMatrix img_cross_3x3_kernel = {(uint8_t[]){0, 1, 0, 1, 1, 1, 0, 1, 0}, 3, 3};

// filters (they also work inplace)
void img_convolution_filter(ImageMatrix* dst, const ImageMatrix src, const ImageMatrixInt8 kernel);
void img_max_filter(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel);
void img_min_filter(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel);
void img_median_filter(ImageMatrix* dst, const ImageMatrix src, ImageMatrix window);

// interpolation methods
uint8_t img_nearest_interpolation(const ImageMatrix mat, Vector2f position);
uint8_t img_bilinear_interpolation(const ImageMatrix mat, Vector2f position);
uint8_t img_bicubic_interpolation(const ImageMatrix mat, Vector2f position);

// image transformations
void img_resize(ImageMatrix dst, const ImageMatrix src, ImageInterpolation interpolation);
void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, uint8_t bg_fill,
        ImageInterpolation interpolation);
void img_affine_transform(ImageMatrix dst, const ImageMatrix src, Matrix2f transform,
        uint8_t bg_fill, ImageInterpolation interpolation);

// histogram thresholding
void img_histogram(uint32_t histogram[256], const ImageMatrix mat);
uint8_t img_compute_otsu_threshold(const uint32_t histogram[256]);

// drawing utilitites
void img_draw_line(ImageMatrix mat, ImagePoint from, ImagePoint to, uint8_t color);
void img_draw_rectangle(ImageMatrix mat, ImagePoint from, ImagePoint to, uint8_t color);

// shape detection
void img_hough_line_transform(ImageMatrixInt32 dst, const ImageMatrix src);

// format conversions
void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src);

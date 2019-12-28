#pragma once
#include "math_utils.h"

typedef struct {
    uint8_t* data;
    int16_t n_cols;
    int16_t n_rows;
} ImageMatrix;

typedef struct {
    float* data;
    int16_t n_cols;
    int16_t n_rows;
} ImageMatrixFloat;

static const int8_t edge_detect_kernel_data[3][3] = {{-1, -1, -1}, {-1, 8, -1}, {-1, -1, -1}};
static const int8_t sobel_x_kernel_data[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
static const int8_t sobel_y_kernel_data[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};

static const ImageMatrix edge_detect_kernel = {(uint8_t*) edge_detect_kernel_data, 3, 3};
static const ImageMatrix sobel_x_kernel = {(uint8_t*) sobel_x_kernel_data, 3, 3};
static const ImageMatrix sobel_y_kernel = {(uint8_t*) sobel_y_kernel_data, 3, 3};

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

#define IMG_APPLY_KERNEL(RET_VAL, KERNEL, MAT, ROW, COL)                                     \
    FOR_EACH_PIXEL_PREFIXED_INDEX(KERNEL, k_) {                                              \
        (RET_VAL) += PIXEL(KERNEL, k_row, k_col) * PIXEL(MAT, k_row + (ROW), k_col + (COL)); \
    }

uint8_t img_average(const ImageMatrix mat);
void img_threshold(ImageMatrix* dst, const ImageMatrix src, uint8_t threshold);
void img_normalize(ImageMatrix* dst, const ImageMatrix src);
void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, uint8_t bg_fill);
void img_draw_line(ImageMatrix mat, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);

void img_convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel);
void img_edge_filter(ImageMatrix* dst, const ImageMatrix src);

void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src);

uint32_t img_count_negative(ImageMatrixFloat mat);
void img_hough_line_transform(ImageMatrixFloat dst, const ImageMatrix src);

#pragma once
#include "math_utils.h"

//#define PIXEL_TYPE int16_t
#ifndef PIXEL_TYPE
#define PIXEL_TYPE uint8_t
#endif

typedef struct {
    PIXEL_TYPE* data;
    int32_t n_cols;
    int32_t n_rows;
} ImageMatrix;

static const int8_t edge_detect_kernel[3 * 3] = {-1, -1, -1, -1, 8, -1, -1, -1, -1};
static const int8_t sobel_kernel_x[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
static const int8_t sobel_kernel_y[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

#define PIXEL(MATRIX, ROW, COL) ((MATRIX).data[(ROW) * (MATRIX).n_cols + (COL)])

#define FOR_EACH_PIXEL(MAT, CH)                                  \
    for (int16_t CH##row = 0; CH##row < (MAT).n_rows; ++CH##row) \
        for (int16_t CH##col = 0; CH##col < (MAT).n_cols; ++CH##col)

static inline void img_copy_dimensions(ImageMatrix* dst, const ImageMatrix src, int16_t border) {
    dst->n_cols = src.n_cols + border;
    dst->n_rows = src.n_rows + border;
}

static inline void img_fill(ImageMatrix mat, PIXEL_TYPE value) {
    FOR_EACH_PIXEL(mat, ) { PIXEL(mat, row, col) = value; }
}

static inline PIXEL_TYPE img_average(const ImageMatrix mat) {
    int32_t sum = 0;
    FOR_EACH_PIXEL(mat, ) { sum += PIXEL(mat, row, col); }
    return sum / (mat.n_rows * mat.n_cols);
}

static inline void img_copy(ImageMatrix* dst, const ImageMatrix src) {
    img_copy_dimensions(dst, src, 0);
    FOR_EACH_PIXEL(src, ) { PIXEL(*dst, row, col) = PIXEL(src, row, col); }
}

static inline void img_threshold(ImageMatrix* dst, const ImageMatrix src, PIXEL_TYPE threshold) {
    img_copy_dimensions(dst, src, 0);
    FOR_EACH_PIXEL(src, ) {
        PIXEL(*dst, row, col) = (PIXEL(src, row, col) >= threshold) * UINT8_MAX;
    }
}

static inline int32_t img_apply_kernel(
        ImageMatrix mat, const int8_t* kernel, uint8_t kernel_size, int16_t row, int16_t col) {
    int32_t sum = 0;
    ImageMatrix bounds = {0, kernel_size, kernel_size};
    FOR_EACH_PIXEL(bounds, k_) {
        sum += kernel[(k_row * kernel_size) + k_col] * PIXEL(mat, k_row + row, k_col + col);
    }
    return sum;
}

static inline void img_edge_filter(ImageMatrix* dst, const ImageMatrix src) {
    img_copy_dimensions(dst, src, -2);
    FOR_EACH_PIXEL(*dst, ) {
        int32_t val = img_apply_kernel(src, edge_detect_kernel, 3, row, col);
        PIXEL(*dst, row, col) = CLAMP(val, 0, INT_TYPE_MAX(PIXEL_TYPE));
    }
}

void img_normalize(ImageMatrix* dst, const ImageMatrix src);
void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, PIXEL_TYPE bg_fill);
void img_draw_line(
        ImageMatrix mat, int16_t x0, int16_t y0, int16_t x1, int16_t y1, PIXEL_TYPE color);

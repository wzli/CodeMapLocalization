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

extern const int8_t edge_detect_kernel[3 * 3];
extern const int8_t sobel_kernel_x[3 * 3];
extern const int8_t sobel_kernel_y[3 * 3];

#define PIXEL(MATRIX, ROW, COL) ((MATRIX).data[(ROW) * (MATRIX).n_cols + (COL)])

#define FOR_EACH_PIXEL(MAT, CH)                                  \
    for (int16_t CH##row = 0; CH##row < (MAT).n_rows; ++CH##row) \
        for (int16_t CH##col = 0; CH##col < (MAT).n_cols; ++CH##col)

static inline int32_t img_apply_kernel(
        ImageMatrix mat, const int8_t* kernel, uint8_t kernel_size, int16_t row, int16_t col) {
    int32_t sum = 0;
    ImageMatrix bounds = {0, kernel_size, kernel_size};
    FOR_EACH_PIXEL(bounds, k_) {
        sum += kernel[(k_row * kernel_size) + k_col] * PIXEL(mat, k_row + row, k_col + col);
    }
    return sum;
}

PIXEL_TYPE img_average(const ImageMatrix mat);
void img_copy(ImageMatrix* dst, const ImageMatrix src);
void img_fill(ImageMatrix mat, PIXEL_TYPE value);
void img_threshold(ImageMatrix* dst, const ImageMatrix src, PIXEL_TYPE threshold);
void img_normalize(ImageMatrix* dst, const ImageMatrix src);
void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, PIXEL_TYPE bg_fill);
void img_edge_filter(ImageMatrix* dst, const ImageMatrix src);

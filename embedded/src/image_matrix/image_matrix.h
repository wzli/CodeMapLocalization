#pragma once
#include "math_utils.h"

typedef struct {
    int16_t* data;
    int16_t n_cols;
    int16_t n_rows;
} ImageMatrix;

#define ELEMENT(MATRIX, ROW, COL) ((MATRIX).data[(ROW) * (MATRIX).n_cols + (COL)])

#define FOR_EACH_ELEMENT(MAT)                        \
    for (int16_t row = 0; row < (MAT).n_rows; ++row) \
        for (int16_t col = 0; col < (MAT).n_cols; ++col)

#define FOR_EACH_GRADIENT(MAT, STATEMENT)                                   \
    for (int16_t row = 0; row < (MAT).n_rows - 2; row++)                    \
        for (int16_t col = 0; col < (MAT).n_cols - 2; col++) {              \
            Vector2f gradient = {};                                         \
            for (int16_t k_row = 0; k_row < 3; k_row++)                     \
                for (int16_t k_col = 0; k_col < 3; k_col++) {               \
                    gradient.x += sobel_kernel_x[(k_row * 3) + k_col] *     \
                                  ELEMENT((MAT), k_row + row, k_col + col); \
                    gradient.y += sobel_kernel_y[(k_row * 3) + k_col] *     \
                                  ELEMENT((MAT), k_row + row, k_col + col); \
                }                                                           \
            (STATEMENT);                                                    \
        }

static const int16_t sobel_kernel_x[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
static const int16_t sobel_kernel_y[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

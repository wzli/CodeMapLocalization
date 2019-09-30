#include "imgproc.h"
#include <stdio.h>
#include <math.h>

#define ELEMENT(MATRIX, ROW, COL) ((MATRIX).data[(ROW) * (MATRIX).n_cols + (COL)])

#define FOR_EACH_ELEMENT(MAT, EXPRESSION)                      \
    do {                                                       \
        for (int16_t row = 0; row < (MAT).n_rows; ++row) {     \
            for (int16_t col = 0; col < (MAT).n_cols; ++col) { \
                (EXPRESSION);                                  \
            }                                                  \
        }                                                      \
    } while (0)

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

static int16_t buf_data[30 * 30];
static int16_t sobel_kernel_x[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
static int16_t sobel_kernel_y[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};

void convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    dst->n_rows = src.n_rows - ((kernel.n_rows >> 1) << 1);
    dst->n_cols = src.n_cols - ((kernel.n_cols >> 1) << 1);
    FOR_EACH_ELEMENT(*dst, ELEMENT(*dst, row, col) = 0);
    for (int16_t k_row = 0; k_row < kernel.n_rows; ++k_row) {
        for (int16_t k_col = 0; k_col < kernel.n_cols; ++k_col) {
            FOR_EACH_ELEMENT(*dst,
                    ELEMENT(*dst, row, col) += ELEMENT(kernel, k_row, k_col) * ELEMENT(src, row + k_row, col + k_col));
        }
    }
}

void edge_filter(ImageMatrix* dst, const ImageMatrix src) {
    // compute x gradient
    ImageMatrix buf_mat = {buf_data, 0, 0};
    ImageMatrix kernel = {sobel_kernel_x, 3, 3};
    convolution(&buf_mat, src, kernel);
    // compute y gradient
    kernel.data = sobel_kernel_y;
    convolution(dst, src, kernel);
    // sum absolute value of gradient components
    FOR_EACH_ELEMENT(
            buf_mat, ELEMENT(*dst, row, col) = (ABS(ELEMENT(*dst, row, col)) + ABS(ELEMENT(buf_mat, row, col))));
}

void convert_uint8_to_int16(ImageMatrix mat) {
    uint8_t* data_uint8 = (uint8_t*) mat.data;
    for (int32_t i = mat.n_rows * mat.n_cols - 1; i >= 0; --i) {
        mat.data[i] = data_uint8[i];
    }
};

void convert_int16_to_uint8(ImageMatrix mat) {
    uint8_t* data_uint8 = (uint8_t*) mat.data;
    int32_t data_len = mat.n_rows * mat.n_cols;
    for (int32_t i = 0; i < data_len; ++i) {
        data_uint8[i] = mat.data[i];
    }
};

void print_matrix(ImageMatrix src) {
    for (int16_t row = 0; row < src.n_rows; ++row) {
        for (int16_t col = 0; col < src.n_cols; ++col) {
            printf("%6d ", ELEMENT(src, row, col));
        }
        puts("");
    }
    puts("");
}

void test(ImageMatrix* dst, const ImageMatrix src) {
    convert_uint8_to_int16(src);
    edge_filter(dst, src);
    int16_t max_element = 0;
    FOR_EACH_ELEMENT(*dst, max_element = MAX(max_element, ELEMENT(*dst, row, col)));
    FOR_EACH_ELEMENT(*dst, ELEMENT(*dst, row, col) = (ELEMENT(*dst, row, col) * 255) / max_element);
    print_matrix(src);
    print_matrix(*dst);
    convert_int16_to_uint8(*dst);
}

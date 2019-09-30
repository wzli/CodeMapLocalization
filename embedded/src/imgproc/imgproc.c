#include "imgproc.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

#define ELEMENT(MATRIX, ROW, COL) \
    ((MATRIX).data[(ROW) * (MATRIX).n_cols + (COL)])

#define FOR_EACH_ELEMENT(MAT, EXPRESSION)                      \
    do {                                                       \
        for (int16_t row = 0; row < (MAT).n_rows; ++row) {     \
            for (int16_t col = 0; col < (MAT).n_cols; ++col) { \
                (EXPRESSION);                                  \
            }                                                  \
        }                                                      \
    } while (0)

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define SQUARE(x) ((x) * (x))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static int16_t buf_data[128 * 128];
static int16_t sobel_kernel_x[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
static int16_t sobel_kernel_y[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};

int32_t count_negative_elements(ImageMatrix mat) {
    int32_t count = 0;
    FOR_EACH_ELEMENT(mat, count += (ELEMENT(mat, row, col) < 0));
    return count;
}

void convolution(
        ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    dst->n_rows = src.n_rows - ((kernel.n_rows >> 1) << 1);
    dst->n_cols = src.n_cols - ((kernel.n_cols >> 1) << 1);
    FOR_EACH_ELEMENT(*dst, ELEMENT(*dst, row, col) = 0);
    for (int16_t k_row = 0; k_row < kernel.n_rows; ++k_row) {
        for (int16_t k_col = 0; k_col < kernel.n_cols; ++k_col) {
            FOR_EACH_ELEMENT(*dst, ELEMENT(*dst, row, col) +=
                                   ELEMENT(kernel, k_row, k_col) *
                                   ELEMENT(src, row + k_row, col + k_col));
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
            buf_mat, ELEMENT(*dst, row, col) =
                             (SQUARE(ELEMENT(*dst, row, col)) +
                                     SQUARE(ELEMENT(buf_mat, row, col))) >>
                             6);
    assert(count_negative_elements(*dst) == 0);
}

void hough_line_transform(ImageMatrix* dst, const ImageMatrix src) {
    FOR_EACH_ELEMENT(*dst, ELEMENT(*dst, row, col) = 0);
    float angle_resolution = M_PI * 0.5f / dst->n_rows;
    float scale_to_index = dst->n_cols / sqrtf((src.n_rows * src.n_rows) +
                                                 (src.n_cols * src.n_cols));
    for (uint16_t i = 0; i < dst->n_rows; ++i) {
        float sin = sinf(i * angle_resolution);
        float cos = cosf(i * angle_resolution);
        FOR_EACH_ELEMENT(src,
                ELEMENT(*dst, i,
                        (int16_t)(((sin * row) + (cos * col)) *
                                  scale_to_index)) += ELEMENT(src, row, col));
    }
    assert(count_negative_elements(*dst) == 0);
};

void normalize_elements(ImageMatrix mat, int16_t max_val) {
    int16_t max_element = 1;
    FOR_EACH_ELEMENT(
            mat, max_element = MAX(max_element, ELEMENT(mat, row, col)));
    FOR_EACH_ELEMENT(
            mat, ELEMENT(mat, row, col) =
                         (ELEMENT(mat, row, col) * max_val) / max_element);
};

void square_elements(ImageMatrix mat) {
    FOR_EACH_ELEMENT(
            mat, ELEMENT(mat, row, col) = SQUARE(ELEMENT(mat, row, col)) >> 8);
};

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

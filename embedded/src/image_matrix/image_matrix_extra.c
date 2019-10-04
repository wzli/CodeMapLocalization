#include "image_matrix_extra.h"

#include <limits.h>
#include <assert.h>

#include <stdio.h>

int32_t count_negative_elements(ImageMatrix mat) {
    int32_t count = 0;
    FOR_EACH_ELEMENT(mat) { count += ELEMENT(mat, row, col) < 0; }
    return count;
}

void convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    dst->n_rows = src.n_rows - (kernel.n_rows & ~1);
    dst->n_cols = src.n_cols - (kernel.n_cols & ~1);
    FOR_EACH_ELEMENT(*dst) { ELEMENT(*dst, row, col) = 0; }
    for (int16_t k_row = 0; k_row < kernel.n_rows; ++k_row) {
        for (int16_t k_col = 0; k_col < kernel.n_cols; ++k_col) {
            FOR_EACH_ELEMENT(*dst) {
                ELEMENT(*dst, row, col) +=
                        ELEMENT(kernel, k_row, k_col) * ELEMENT(src, row + k_row, col + k_col);
            }
        }
    }
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

void edge_filter(ImageMatrix* dst, const ImageMatrix src) {
    dst->n_rows = src.n_rows - 2;
    dst->n_cols = src.n_cols - 2;
    float norm_scale = 1.0f / (1 << 6);
    FOR_EACH_GRADIENT(src, ELEMENT(*dst, row, col) = v2f_norm_sqr(gradient) * norm_scale);
    assert(count_negative_elements(*dst) == 0);
}

void hough_line_transform(ImageMatrix dst, const ImageMatrix src) {
    FOR_EACH_ELEMENT(dst) { ELEMENT(dst, row, col) = 0; }
    float angle_resolution = M_PI * 0.5f / dst.n_rows;
    float scale_to_index =
            dst.n_cols / sqrtf((src.n_rows * src.n_rows) + (src.n_cols * src.n_cols));
    for (uint16_t i = 0; i < dst.n_rows; ++i) {
        float sin = sinf(i * angle_resolution);
        float cos = cosf(i * angle_resolution);
        FOR_EACH_ELEMENT(src) {
            ELEMENT(dst, i, (int16_t)(((sin * row) + (cos * col)) * scale_to_index)) +=
                    ELEMENT(src, row, col);
        }
    }
    assert(count_negative_elements(dst) == 0);
};

void print_bits(uint32_t word, int8_t word_length) {
    for (word_length--; word_length >= 0; word_length--) {
        printf("%u", (word >> word_length) & 1);
    }
    puts("");
}

void print_matrix(ImageMatrix src) {
    for (int16_t row = 0; row < src.n_rows; ++row) {
        for (int16_t col = 0; col < src.n_cols; ++col) {
            printf("%6d ", ELEMENT(src, row, col));
        }
        puts("");
    }
    puts("");
}

void threshold_elements(ImageMatrix mat, int16_t threshold, int16_t max_val) {
    FOR_EACH_ELEMENT(mat) {
        ELEMENT(mat, row, col) = (ELEMENT(mat, row, col) >= threshold) * max_val;
    }
}

void normalize_elements(ImageMatrix mat, int16_t max_val) {
    int16_t max_element = SHRT_MIN;
    int16_t min_element = SHRT_MAX;
    FOR_EACH_ELEMENT(mat) {
        max_element = MAX(max_element, ELEMENT(mat, row, col));
        min_element = MIN(min_element, ELEMENT(mat, row, col));
    }
    if (max_element == min_element) {
        return;
    }
    FOR_EACH_ELEMENT(mat) {
        ELEMENT(mat, row, col) =
                ((ELEMENT(mat, row, col) - min_element) * max_val) / (max_element - min_element);
    }
};

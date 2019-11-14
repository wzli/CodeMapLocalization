#include "code_extraction.h"
#include <assert.h>

Vector2f img_estimate_rotation(const ImageMatrix mat) {
    Vector2f gradient_sum = {};
    FOR_EACH_GRADIENT(mat) {
        gradient_sum = v2f_add(gradient_sum, v2f_double_angle(v2f_double_angle(gradient)));
    }
    if (!v2f_is_zero(gradient_sum)) {
        gradient_sum = v2f_normalize(gradient_sum);
        gradient_sum = v2f_half_angle(gradient_sum);
        gradient_sum = v2f_half_angle(gradient_sum);
        assert(!v2f_is_nan(gradient_sum));
    }
    return gradient_sum;
}

void img_bit_matrix_conversion(BitMatrix32 dst, BitMatrix32 mask, const ImageMatrix src,
        IMG_TYPE low_thresh, IMG_TYPE high_thresh) {
    assert(src.n_rows == 32 && src.n_cols == 32);
    assert(high_thresh >= low_thresh);
    for (uint8_t row = 0; row < 32; ++row) {
        dst[row] = 0;
        mask[row] = ~0;
    }
    FOR_EACH_ELEMENT(src) {
        if (ELEMENT(src, row, col) >= high_thresh) {
            bm32_set_bit(dst, row, col);
        } else if (ELEMENT(src, row, col) > low_thresh) {
            bm32_clear_bit(mask, row, col);
        }
    }
}

AxisCode bm32_extract_column_code(uint32_t initial_row_guess, const BitMatrix32 matrix,
        const BitMatrix32 mask, uint8_t min_row_samples) {
    assert(min_row_samples > 0);
    assert(matrix && mask);
    if (!initial_row_guess) {
        initial_row_guess = matrix[15] & mask[15];
    }
    static const uint8_t FILTER_SIZE = 4;
    uint32_t row_code_bits[FILTER_SIZE];
    for (int j = 0; j < FILTER_SIZE; ++j) {
        row_code_bits[j] = initial_row_guess;
    }
    AxisCode column_code = {};
    for (uint8_t i = 0; i < 32; ++i) {
        uint8_t mask_sum = sum_bits(mask[i]);
        if (mask_sum < min_row_samples) {
            continue;
        }
        mask_sum *= FILTER_SIZE;
        uint8_t row_diff = 0;
        for (int j = 0; j < FILTER_SIZE; ++j) {
            row_diff += sum_bits((row_code_bits[j] ^ matrix[i]) & mask[i]);
        }
        uint8_t j = i & (FILTER_SIZE - 1);
        row_code_bits[j] &= ~mask[i];
        if (row_diff > mask_sum / 2) {
            column_code.bits |= 1 << i;
            row_code_bits[j] |= ~matrix[i] & mask[i];
            column_code.n_errors += mask_sum - row_diff;
        } else {
            row_code_bits[j] |= matrix[i] & mask[i];
            column_code.n_errors += row_diff;
        }
        column_code.n_samples += mask_sum;
        column_code.mask |= 1 << i;
    }
    return column_code;
}

void bm32_extract_axis_codes(AxisCode* row_code, AxisCode* col_code, BitMatrix32 matrix,
        BitMatrix32 mask, uint8_t min_samples) {
    assert(row_code && col_code);
    *col_code = bm32_extract_column_code(0, matrix, mask, min_samples);
    bm32_transpose(matrix);
    bm32_transpose(mask);
    *row_code = bm32_extract_column_code(col_code->bits, matrix, mask, min_samples);
    uint8_t offset = first_set_bit(col_code->mask);
    col_code->bits >>= offset;
    col_code->mask >>= offset;
    offset = first_set_bit(row_code->mask);
    row_code->bits >>= offset;
    row_code->mask >>= offset;
}

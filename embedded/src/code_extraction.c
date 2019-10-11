#include "code_extraction.h"
#include <assert.h>

Vector2f img_estimate_rotation(const ImageMatrix mat) {
    Vector2f gradient_sum = {};
    FOR_EACH_GRADIENT(mat,
            gradient_sum = v2f_add(gradient_sum, v2f_double_angle(v2f_double_angle(gradient))));
    if (!v2f_is_zero(gradient_sum)) {
        gradient_sum = v2f_normalize(gradient_sum);
        gradient_sum = v2f_half_angle(gradient_sum);
        gradient_sum = v2f_half_angle(gradient_sum);
        assert(!v2f_is_nan(gradient_sum));
    }
    return gradient_sum;
}

void img_bit_matrix_conversion(BitMatrix32 dst, BitMatrix32 mask, const ImageMatrix src,
        IMF_TYPE low_thresh, IMF_TYPE high_thresh) {
    assert(src.n_rows == 32 && src.n_cols == 32);
    assert(high_thresh >= low_thresh);
    FOR_EACH_ELEMENT(src) {
        if (ELEMENT(src, row, col) >= high_thresh) {
            bm32_set_bit(dst, row, col);
        } else if (ELEMENT(src, row, col) <= low_thresh) {
            bm32_clear_bit(dst, row, col);
        } else {
            bm32_clear_bit(mask, row, col);
            continue;
        }
        bm32_set_bit(mask, row, col);
    }
}

AxisCode bm32_extract_column_code(
        uint32_t row_code_bits, const BitMatrix32 matrix, const BitMatrix32 mask) {
    AxisCode column_code = {};
    for (uint8_t i = 0; i < 32; ++i) {
        uint8_t mask_sum = sum_bits(mask[i]);
        if (mask_sum == 0) {
            continue;
        }
        uint8_t row_diff = sum_bits((row_code_bits ^ matrix[i]) & mask[i]);
        uint32_t inv_bits = -(2 * row_diff > mask_sum);
        column_code.bits |= (inv_bits & 1) << i;
        column_code.mask |= 1 << i;
        row_code_bits &= ~mask[i];
        row_code_bits |= (inv_bits ^ matrix[i]) & mask[i];
    }
    return column_code;
}

void bm32_extract_axis_codes(
        AxisCode* row_code, AxisCode* col_code, BitMatrix32 matrix, BitMatrix32 mask) {
    assert(row_code && col_code);
    *col_code = bm32_extract_column_code(0, matrix, mask);
    bm32_transpose32(matrix);
    bm32_transpose32(mask);
    *row_code = bm32_extract_column_code(col_code->bits, matrix, mask);
    uint8_t offset = first_set_bit(col_code->mask);
    col_code->bits >>= offset;
    col_code->mask >>= offset;
    offset = first_set_bit(row_code->mask);
    row_code->bits >>= offset;
    row_code->mask >>= offset;
}

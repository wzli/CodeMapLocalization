#include "code_extraction.h"
#include <assert.h>

void img_edge_hysteresis_threshold(ImageMatrix* dst, const ImageMatrix src, uint16_t edge_thresh,
        uint8_t value_thresh, uint8_t value_tolerance) {
    uint8_t hi_thresh = MAX(0, value_thresh - value_tolerance);
    uint8_t lo_thresh = MIN(UINT8_MAX, value_thresh + value_tolerance);
    uint16_t latched_value = UINT8_MAX / 2;
    dst->n_rows = src.n_rows - 2;
    dst->n_cols = src.n_cols - 2;
    FOR_EACH_PIXEL(*dst) {
        int16_t s_col = row & 1 ? dst->n_cols - col - 1 : col;
        int32_t edge = 0;
        IMG_APPLY_KERNEL(edge, edge_kernel, src, row, s_col);
        if (edge < -edge_thresh && PIXEL(src, row, s_col) < lo_thresh) {
            latched_value = 0;
        } else if (edge > edge_thresh && PIXEL(src, row, s_col) > hi_thresh) {
            latched_value = UINT8_MAX;
        } else if (PIXEL(src, row, s_col) == value_thresh) {
            latched_value = UINT8_MAX / 2;
        } else if (row > 0) {
            uint8_t count = 1;
            if (s_col > 0) {
                latched_value += PIXEL(*dst, row - 1, s_col - 1);
                ++count;
            }
            if (s_col < dst->n_cols - 1) {
                latched_value += PIXEL(*dst, row - 1, s_col + 1);
                ++count;
            }
            if (col > 0) {
                latched_value += PIXEL(*dst, row - 1, s_col);
                ++count;
            }
            assert(count > 1);
            latched_value = ((2 * latched_value) >= (count * UINT8_MAX)) * UINT8_MAX;
        }
        PIXEL(*dst, row, s_col) = latched_value;
    }
}

Vector2f img_estimate_rotation(const ImageMatrix mat) {
    Vector2f gradient_sum = {};
    ImageMatrix bounds = {0, mat.n_cols - 2, mat.n_rows - 2};
    FOR_EACH_PIXEL(bounds) {
        int32_t grad_x = 0, grad_y = 0;
        IMG_APPLY_KERNEL(grad_x, sobel_x_kernel, mat, row, col);
        IMG_APPLY_KERNEL(grad_y, sobel_y_kernel, mat, row, col);
        Vector2f gradient = {grad_x, grad_y};
        gradient = v2f_double_angle(gradient);
        gradient = v2f_double_angle(gradient);
        gradient_sum = v2f_add(gradient_sum, gradient);
    }
    if (!v2f_is_zero(gradient_sum)) {
        gradient_sum = v2f_normalize(gradient_sum);
        gradient_sum = v2f_half_angle(gradient_sum);
        gradient_sum = v2f_half_angle(gradient_sum);
        assert(!v2f_is_nan(gradient_sum));
    }
    return gradient_sum;
}

float img_estimate_scale(const ImageMatrix mat) {
    int32_t sum = 0, max_val = 0;
    ImageMatrix bounds = {0, mat.n_cols - 2, mat.n_rows - 2};
    FOR_EACH_PIXEL(bounds) {
        int32_t val = 0;
        IMG_APPLY_KERNEL(val, edge_kernel, mat, row, col);
        if (val > 0) {
            sum += val;
            max_val = MAX(max_val, val);
        }
    }
    assert(sum >= 0);
    return bounds.n_rows * bounds.n_cols * max_val / (2 * sum + 0.00001f) - 1;
}

void img_bit_matrix_conversion(BitMatrix32 dst, BitMatrix32 mask, const ImageMatrix src,
        uint8_t low_thresh, uint8_t high_thresh) {
    assert(src.n_rows == 32 && src.n_cols == 32);
    assert(high_thresh >= low_thresh);
    for (uint8_t row = 0; row < 32; ++row) {
        dst[row] = 0;
        mask[row] = ~0;
    }
    FOR_EACH_PIXEL(src) {
        if (PIXEL(src, row, col) >= high_thresh) {
            bm32_set_bit(dst, row, col);
        } else if (PIXEL(src, row, col) > low_thresh) {
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
    for (uint8_t j = 0; j < FILTER_SIZE; ++j) {
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
        for (uint8_t j = 0; j < FILTER_SIZE; ++j) {
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

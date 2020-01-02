#include "code_extraction.h"
#include <assert.h>

Vector2f img_estimate_rotation(const ImageMatrix mat) {
    Vector2f gradient_sum = {};
    ImageMatrix bounds = {0, mat.n_cols - 2, mat.n_rows - 2};
    FOR_EACH_PIXEL(bounds) {
        Vector2f gradient = {
                PIXEL(mat, row, col + 2) - PIXEL(mat, row, col) +
                        2 * (PIXEL(mat, row + 1, col + 2) - PIXEL(mat, row + 1, col)) +
                        PIXEL(mat, row + 2, col + 2) - PIXEL(mat, row + 2, col),
                PIXEL(mat, row + 2, col) - PIXEL(mat, row, col) +
                        2 * (PIXEL(mat, row + 2, col + 1) - PIXEL(mat, row, col + 1)) +
                        PIXEL(mat, row + 2, col + 2) - PIXEL(mat, row, col + 2),
        };
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
        IMG_APPLY_KERNEL(val, img_edge_detect_kernel, mat, row, col);
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

void img_to_bm64(BitMatrix64 dst, BitMatrix64 mask, const ImageMatrix src, uint8_t low_thresh,
        uint8_t high_thresh) {
    assert(high_thresh >= low_thresh);
    for (uint8_t row = 0; row < 64; ++row) {
        dst[row] = 0ull;
        mask[row] = 0ull;
    }
    FOR_EACH_PIXEL(src) {
        if (row >= 64 || col >= 64) {
            break;
        } else if (PIXEL(src, row, col) > high_thresh) {
            bm64_set_bit(dst, row, col);
            bm64_set_bit(mask, row, col);
        } else if (PIXEL(src, row, col) < low_thresh) {
            bm64_set_bit(mask, row, col);
        }
    }
}

void bm64_to_img(ImageMatrix* dst, const BitMatrix64 src, const BitMatrix64 mask) {
    IMG_SET_SIZE(dst, 64, 64);
    FOR_EACH_PIXEL(*dst) {
        PIXEL(*dst, row, col) = bm64_get_bit(mask, row, col)
                                        ? bm64_get_bit(src, row, col) * UINT8_MAX
                                        : UINT8_MAX / 2;
    }
}

AxisCode bm32_extract_column_code(uint32_t row_estimate, const BitMatrix32 matrix,
        const BitMatrix32 mask, uint8_t min_row_samples) {
    assert(matrix && mask);
    AxisCode column_code = {};
    int8_t bit_votes[32] = {};
    for (uint8_t i = 0; i < 32; ++i) {
        uint8_t mask_sum = count_bits(mask[i]);
        if (mask_sum < min_row_samples) {
            continue;
        }
        uint8_t row_diff = count_bits((row_estimate ^ matrix[i]) & mask[i]);
        row_estimate &= ~mask[i];
        if (2 * row_diff > mask_sum) {
            row_estimate |= ~matrix[i] & mask[i];
            column_code.bits |= 1 << i;
            column_code.n_errors += mask_sum - row_diff;
        } else {
            row_estimate |= matrix[i] & mask[i];
            column_code.n_errors += row_diff;
        }
        column_code.n_samples += mask_sum;
        column_code.mask |= 1 << i;
        for (uint8_t j = 0; j < 32; ++j) {
            if (!bm32_get_bit(mask, i, j)) {
                continue;
            }
            bit_votes[j] += (row_estimate >> j) & 1 ? 1 : -1;
            row_estimate &= ~(1 << j);
            row_estimate |= ((bit_votes[j]) > 0) << j;
        }
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
    uint8_t offset = count_trailing_zeros(col_code->mask);
    col_code->bits >>= offset;
    col_code->mask >>= offset;
    offset = count_trailing_zeros(row_code->mask);
    row_code->bits >>= offset;
    row_code->mask >>= offset;
}

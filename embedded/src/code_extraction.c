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

#define IMPLEMENT_IMG_TO_BM(W)                                                    \
    void img_to_bm##W(BitMatrix##W dst, BitMatrix##W mask, const ImageMatrix src, \
            uint8_t low_thresh, uint8_t high_thresh) {                            \
        assert(high_thresh >= low_thresh);                                        \
        for (uint8_t row = 0; row < (W); ++row) {                                 \
            dst[row] = 0;                                                         \
            mask[row] = 0;                                                        \
        }                                                                         \
        FOR_EACH_PIXEL(src) {                                                     \
            if (row >= (W) || col >= (W)) {                                       \
                break;                                                            \
            } else if (PIXEL(src, row, col) > high_thresh) {                      \
                bm##W##_set_bit(dst, row, col);                                   \
                bm##W##_set_bit(mask, row, col);                                  \
            } else if (PIXEL(src, row, col) < low_thresh) {                       \
                bm##W##_set_bit(mask, row, col);                                  \
            }                                                                     \
        }                                                                         \
    }

IMPLEMENT_IMG_TO_BM(32);
IMPLEMENT_IMG_TO_BM(64);

#define IMPLEMENT_BM_TO_IMG(W)                                                               \
    void bm##W##_to_img(ImageMatrix* dst, const BitMatrix##W src, const BitMatrix##W mask) { \
        IMG_SET_SIZE(*dst, (W), (W));                                                        \
        FOR_EACH_PIXEL(*dst) {                                                               \
            PIXEL(*dst, row, col) = bm##W##_get_bit(mask, row, col)                          \
                                            ? bm##W##_get_bit(src, row, col) * UINT8_MAX     \
                                            : UINT8_MAX / 2;                                 \
        }                                                                                    \
    }

IMPLEMENT_BM_TO_IMG(32);
IMPLEMENT_BM_TO_IMG(64);

#define IMPLEMENT_BM_FROM_AXIS_CODES(W)                                                           \
    void bm##W##_from_axis_codes(BitMatrix##W dst, BitMatrix##W mask, const AxisCode##W row_code, \
            const AxisCode##W col_code) {                                                         \
        for (uint8_t i = 0; i < (W); ++i) {                                                       \
            dst[i] = ((col_code.bits >> i) & 1) ? ~row_code.bits : row_code.bits;                 \
            mask[i] = ((col_code.mask >> i) & 1) ? row_code.mask : 0;                             \
        }                                                                                         \
    }

IMPLEMENT_BM_FROM_AXIS_CODES(32);
IMPLEMENT_BM_FROM_AXIS_CODES(64);

#define IMPLEMENT_BM_EXTRACT_COLUMN_CODE(W)                                                      \
    AxisCode##W bm##W##_extract_column_code(uint##W##_t row_estimate, const BitMatrix##W matrix, \
            const BitMatrix##W mask, uint8_t min_row_samples) {                                  \
        assert(matrix&& mask);                                                                   \
        AxisCode##W column_code = {};                                                            \
        int8_t bit_votes[(W)] = {};                                                              \
        for (uint8_t i = 0; i < (W); ++i) {                                                      \
            uint8_t mask_sum = count_bits(mask[i]);                                              \
            if (mask_sum < min_row_samples) {                                                    \
                continue;                                                                        \
            }                                                                                    \
            uint8_t row_diff = count_bits((row_estimate ^ matrix[i]) & mask[i]);                 \
            row_estimate &= ~mask[i];                                                            \
            if (2 * row_diff > mask_sum) {                                                       \
                row_estimate |= ~matrix[i] & mask[i];                                            \
                column_code.bits |= (uint##W##_t) 1 << i;                                        \
                column_code.n_errors += mask_sum - row_diff;                                     \
            } else {                                                                             \
                row_estimate |= matrix[i] & mask[i];                                             \
                column_code.n_errors += row_diff;                                                \
            }                                                                                    \
            column_code.n_samples += mask_sum;                                                   \
            column_code.mask |= (uint##W##_t) 1 << i;                                            \
            for (uint8_t j = count_trailing_zeros(mask[i]); j < (W); ++j) {                      \
                if (!bm##W##_get_bit(mask, i, j)) {                                              \
                    continue;                                                                    \
                }                                                                                \
                bit_votes[j] += (row_estimate >> j) & 1 ? 1 : -1;                                \
                row_estimate &= ~((uint##W##_t) 1 << j);                                         \
                row_estimate |= (uint##W##_t)((bit_votes[j]) > 0) << j;                          \
            }                                                                                    \
        }                                                                                        \
        return column_code;                                                                      \
    }

IMPLEMENT_BM_EXTRACT_COLUMN_CODE(32);
IMPLEMENT_BM_EXTRACT_COLUMN_CODE(64);

#define IMPLEMENT_BM_EXTRACT_AXIS_CODES(W)                                                   \
    void bm##W##_extract_axis_codes(AxisCode##W* row_code, AxisCode##W* col_code,            \
            BitMatrix##W matrix, BitMatrix##W mask, uint8_t min_samples) {                   \
        assert(row_code&& col_code);                                                         \
        *col_code = bm##W##_extract_column_code(matrix[(W) / 2], matrix, mask, min_samples); \
        bm##W##_transpose(matrix);                                                           \
        bm##W##_transpose(mask);                                                             \
        *row_code = bm##W##_extract_column_code(col_code->bits, matrix, mask, min_samples);  \
        uint8_t offset = count_trailing_zeros(col_code->mask);                               \
        col_code->bits >>= offset;                                                           \
        col_code->mask >>= offset;                                                           \
        offset = count_trailing_zeros(row_code->mask);                                       \
        row_code->bits >>= offset;                                                           \
        row_code->mask >>= offset;                                                           \
    }

IMPLEMENT_BM_EXTRACT_AXIS_CODES(32);
IMPLEMENT_BM_EXTRACT_AXIS_CODES(64);

AxisCode32 downsample_axis_code(AxisCode64 axis_code_64) {
    uint64_t edges = axis_code_64.bits ^ (axis_code_64.bits >> 1);
    uint8_t offset_index = 0;
    uint8_t lowest_bit_errors = UINT8_MAX;
    for (uint64_t repeating001s = 0x1249249249249249ull; repeating001s & 7; repeating001s <<= 1) {
        uint8_t bit_errors = count_bits((edges ^ repeating001s) & axis_code_64.mask);
        if (bit_errors < lowest_bit_errors) {
            lowest_bit_errors = bit_errors;
            offset_index = count_trailing_zeros((uint32_t) repeating001s);
        }
    }
    if (offset_index < 2) {
        axis_code_64.bits >>= offset_index + 1;
        axis_code_64.mask >>= offset_index + 1;
    }
    AxisCode32 axis_code_32 = {0, 0, axis_code_64.n_errors, axis_code_64.n_samples};
    static const uint8_t count_bits_3[8] = {0, 1, 1, 2, 1, 2, 2, 3};
    uint32_t current_bit = 1;
    while (axis_code_64.mask) {
        if (count_bits_3[axis_code_64.mask & 7] >= 3) {
            axis_code_32.mask |= current_bit;
            if (2 * count_bits_3[axis_code_64.bits & 7] > 3) {
                axis_code_32.bits |= current_bit;
            }
        }
        current_bit <<= 1;
        axis_code_64.bits >>= 3;
        axis_code_64.mask >>= 3;
    }
    return axis_code_32;
};

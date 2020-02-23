#include "code_extraction.h"
#include <assert.h>

#define WIDTH 32
#include "bm_extraction_impl.h"

#define WIDTH 64
#include "bm_extraction_impl.h"

void img_hyper_sharpen(ImageMatrix* dst, const ImageMatrix src) {
    IMG_SET_SIZE(*dst, src.size.x - 2, src.size.y - 2);
    FOR_EACH_PIXEL(*dst) {
        int16_t value = 25 * PIXEL(src, row + 1, col + 1) -
                        3 * (PIXEL(src, row, col) + PIXEL(src, row, col + 1) +
                                    PIXEL(src, row, col + 2) + PIXEL(src, row + 1, col) +
                                    PIXEL(src, row + 1, col + 2) + PIXEL(src, row + 2, col) +
                                    PIXEL(src, row + 2, col + 1) + PIXEL(src, row + 2, col + 2));
        PIXEL(*dst, row, col) = CLAMP(value, 0, UINT8_MAX);
    }
}

Vector2f img_estimate_rotation(const ImageMatrix mat) {
    Vector2f gradient_sum = {};
    ImageMatrix bounds = {0, {mat.size.x - 2, mat.size.y - 2}};
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
    ImageMatrix bounds = {0, {mat.size.x - 2, mat.size.y - 2}};
    FOR_EACH_PIXEL(bounds) {
        int32_t val = 0;
        IMG_PIXEL_WEIGHTED_SUM(val, img_edge_detect_kernel, mat, row, col);
        if (val > 0) {
            sum += val;
            max_val = MAX(max_val, val);
        }
    }
    assert(sum >= 0);
    return bounds.size.y * bounds.size.x * max_val / (2 * sum + 0.00001f) - 1;
}

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

AxisCode64 scale_axis_code(AxisCode64 axis_code, float scale) {
    assert(scale > 0);
    scale = 1.0f / scale;
    AxisCode64 scaled_axis_code = {0, 0, axis_code.n_errors, axis_code.n_samples};
    uint64_t bit = 1;
    for (float index = 0; index < 64; index += scale, bit <<= 1) {
        if ((axis_code.bits >> (uint8_t) index) & 1) {
            scaled_axis_code.bits |= bit;
        }
        if ((axis_code.mask >> (uint8_t) index) & 1) {
            scaled_axis_code.mask |= bit;
        }
    }
    return scaled_axis_code;
}

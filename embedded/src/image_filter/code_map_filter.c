#include "image_filter.h"
#include "bit_matrix/bit_matrix.h"

#include <float.h>
#include <assert.h>

Vector2f cmf_estimate_rotation(const ImageMatrix mat) {
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

void cmf_bit_matrix_conversion(BitMatrix32 dst, BitMatrix32 mask, const ImageMatrix src,
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

#if 0
void cmf_rotated_bit_mask(BitMatrix32 rotated_mask, const ImageMatrix src, Vector2f rotation) {
    assert(!v2f_is_zero(rotation) && !v2f_is_nan(rotation));
    if (rotation.x < 0) {
        rotation = v2f_scale(rotation, -1);
    }
    rotation.x += FLT_EPSILON;
    rotation.y += (rotation.y == 0) * FLT_EPSILON;
    Vector2f rotated_corners[2];
    for (uint8_t i = 0; i < 2; ++i) {
        Vector2f from_center = {(i - 0.5f) * (src.n_cols - 1), (i - 0.5f) * (src.n_rows - 1)};
        rotated_corners[i] = v2f_add(v2f_rotate(from_center, rotation), (Vector2f){16, 16});
    }
    float inv_slope = rotation.x / rotation.y;
    float inv_slope_90 = -rotation.y / rotation.x;
    float x_intercepts[4] = {
            rotated_corners[0].x - inv_slope * (rotated_corners[0].y - 0.5f),
            rotated_corners[1].x - inv_slope_90 * (rotated_corners[1].y + 0.5f - (rotation.y < 0)),
            rotated_corners[1].x - inv_slope * (rotated_corners[1].y + 0.5f),
            rotated_corners[0].x - inv_slope_90 * (rotated_corners[0].y - 0.5f + (rotation.y < 0)),
    };
    for (uint8_t i = 0; i < 32; ++i) {
        float y = (0.5f + i);
        float intercepts[4] = {
                inv_slope * y + x_intercepts[0],
                inv_slope_90 * y + x_intercepts[1],
                inv_slope * y + x_intercepts[2],
                inv_slope_90 * y + x_intercepts[3],
        };
        float range_begin = rotation.y > 0 ? MAX(intercepts[2], intercepts[3])
                                           : MAX(intercepts[0], intercepts[3]);
        float range_end = rotation.y > 0 ? MIN(intercepts[0], intercepts[1])
                                         : MIN(intercepts[1], intercepts[2]);
        range_begin = CLAMP(range_begin, 0, 31);
        range_end = CLAMP(range_end, 0, 31);
        rotated_mask[i] = range_end <= range_begin ? 0u
                                                   : (~0u << (uint8_t) range_begin) &
                                                             (~0u >> (31 - (uint8_t) range_end));
    }
}
#endif

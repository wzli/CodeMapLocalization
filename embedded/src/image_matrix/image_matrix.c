#include "image_matrix.h"
#include "bit_matrix/bit_matrix.h"

#include <float.h>
#include <assert.h>

Vector2f estimate_rotation(const ImageMatrix mat) {
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

void rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation) {
    assert(!v2f_is_zero(rotation) && !v2f_is_nan(rotation));
    assert(!isnan(rotation.x) && !isnan(rotation.y));
    Vector2f src_center = {0.5f * src.n_cols, 0.5f * src.n_rows};
    Vector2f dst_center = {0.5f * dst.n_cols, 0.5f * dst.n_rows};
    rotation = v2f_flip_rotation(rotation);
    FOR_EACH_ELEMENT(dst) {
        Vector2f from_center = {0.5f + col - dst_center.x, 0.5f + row - dst_center.y};
        Vector2f src_position = v2f_add(src_center, v2f_rotate(from_center, rotation));
        if (src_position.x < 0.0f || src_position.x >= src.n_cols || src_position.y < 0.0f ||
                src_position.y >= src.n_rows) {
            continue;
        }
        int16_t right = round(src_position.x);
        int16_t bottom = round(src_position.y);
        int16_t left = MAX(right - 1, 0);
        int16_t top = MAX(bottom - 1, 0);
        right = MIN(right, src.n_cols - 1);
        bottom = MIN(bottom, src.n_rows - 1);
        Vector2f progress = {src_position.x - 0.5f - left, src_position.y - 0.5f - top};
        float top_average = (float) ELEMENT(src, top, left) +
                            progress.x * (ELEMENT(src, top, right) - ELEMENT(src, top, left));
        float bottom_average =
                (float) ELEMENT(src, bottom, left) +
                progress.x * (ELEMENT(src, bottom, right) - ELEMENT(src, bottom, left));
        ELEMENT(dst, row, col) = top_average + progress.y * (bottom_average - top_average);
    }
}

void rotation_bit_mask(BitMatrix32 rotated_mask, const ImageMatrix src, Vector2f rotation) {
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
        range_begin = range_begin < 0 ? 0 : range_begin > 31 ? 31 : range_begin;
        range_end = range_end < 0 ? 0 : range_end > 31 ? 31 : range_end;
        rotated_mask[i] = range_end <= range_begin ? 0u
                                                   : (~0u << (uint8_t) range_begin) &
                                                             (~0u >> (31 - (uint8_t) range_end));
    }
}

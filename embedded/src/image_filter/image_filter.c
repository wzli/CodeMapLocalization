#include "image_filter.h"
#include <assert.h>

const int16_t sobel_kernel_x[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
const int16_t sobel_kernel_y[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

void imf_fill(ImageMatrix mat, IMF_TYPE value) {
    FOR_EACH_ELEMENT(mat) { ELEMENT(mat, row, col) = value; }
}

void imf_threshold(ImageMatrix mat, IMF_TYPE threshold) {
    FOR_EACH_ELEMENT(mat) { ELEMENT(mat, row, col) = (ELEMENT(mat, row, col) >= threshold) * 255; }
}

void imf_normalize(ImageMatrix mat) {
    IMF_TYPE max_element = ELEMENT(mat, 0, 0);
    IMF_TYPE min_element = ELEMENT(mat, 0, 0);
    FOR_EACH_ELEMENT(mat) {
        max_element = MAX(max_element, ELEMENT(mat, row, col));
        min_element = MIN(min_element, ELEMENT(mat, row, col));
    }
    if (max_element == min_element) {
        return;
    }
    FOR_EACH_ELEMENT(mat) {
        ELEMENT(mat, row, col) =
                ((ELEMENT(mat, row, col) - min_element) * 255) / (max_element - min_element);
    }
}

void imf_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, IMF_TYPE bg_fill) {
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
            ELEMENT(dst, row, col) = bg_fill;
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

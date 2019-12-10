#include "image_utils.h"
#include <assert.h>

const int8_t edge_detect_kernel[3 * 3] = {-1, -1, -1, -1, 8, -1, -1, -1, -1};
const int8_t sobel_kernel_x[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
const int8_t sobel_kernel_y[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

void img_copy(ImageMatrix* dst, const ImageMatrix src) {
    dst->n_cols = src.n_cols;
    dst->n_rows = src.n_rows;
    FOR_EACH_PIXEL(src, ) { PIXEL(*dst, row, col) = PIXEL(src, row, col); }
}

void img_fill(ImageMatrix mat, PIXEL_TYPE value) {
    FOR_EACH_PIXEL(mat, ) { PIXEL(mat, row, col) = value; }
}

void img_threshold(ImageMatrix mat, PIXEL_TYPE threshold) {
    FOR_EACH_PIXEL(mat, ) { PIXEL(mat, row, col) = (PIXEL(mat, row, col) >= threshold) * 255; }
}

void img_normalize(ImageMatrix mat) {
    PIXEL_TYPE max_pixel = PIXEL(mat, 0, 0);
    PIXEL_TYPE min_pixel = PIXEL(mat, 0, 0);
    FOR_EACH_PIXEL(mat, ) {
        max_pixel = MAX(max_pixel, PIXEL(mat, row, col));
        min_pixel = MIN(min_pixel, PIXEL(mat, row, col));
    }
    if (max_pixel == min_pixel) {
        return;
    }
    FOR_EACH_PIXEL(mat, ) {
        PIXEL(mat, row, col) = ((PIXEL(mat, row, col) - min_pixel) * 255) / (max_pixel - min_pixel);
    }
}

void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, PIXEL_TYPE bg_fill) {
    assert(!v2f_is_zero(rotation) && !v2f_is_nan(rotation));
    Vector2f src_center = {0.5f * src.n_cols, 0.5f * src.n_rows};
    Vector2f dst_center = {0.5f * dst.n_cols, 0.5f * dst.n_rows};
    rotation = v2f_flip_rotation(rotation);
    FOR_EACH_PIXEL(dst, ) {
        Vector2f from_center = {0.5f + col - dst_center.x, 0.5f + row - dst_center.y};
        Vector2f src_position = v2f_add(src_center, v2f_rotate(from_center, rotation));
        if (src_position.x < 0.0f || src_position.x >= src.n_cols || src_position.y < 0.0f ||
                src_position.y >= src.n_rows) {
            PIXEL(dst, row, col) = bg_fill;
            continue;
        }
        int16_t right = src_position.x + 0.5f;
        int16_t bottom = src_position.y + 0.5f;
        int16_t left = MAX(right - 1, 0);
        int16_t top = MAX(bottom - 1, 0);
        right = MIN(right, src.n_cols - 1);
        bottom = MIN(bottom, src.n_rows - 1);
        Vector2f progress = {src_position.x - 0.5f - left, src_position.y - 0.5f - top};
        float top_average = (float) PIXEL(src, top, left) +
                            progress.x * (PIXEL(src, top, right) - PIXEL(src, top, left));
        float bottom_average = (float) PIXEL(src, bottom, left) +
                               progress.x * (PIXEL(src, bottom, right) - PIXEL(src, bottom, left));
        PIXEL(dst, row, col) = top_average + progress.y * (bottom_average - top_average);
    }
}

void img_edge_filter(ImageMatrix* dst, const ImageMatrix src) {
    dst->n_rows = src.n_rows - 2;
    dst->n_cols = src.n_cols - 2;
    FOR_EACH_PIXEL(*dst, ) {
        int32_t val = img_apply_kernel(src, edge_detect_kernel, 3, row, col);
        PIXEL(*dst, row, col) = CLAMP(val, 0, INT_TYPE_MAX(PIXEL_TYPE));
    }
}

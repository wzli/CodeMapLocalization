#include "image_utils.h"
#include <assert.h>

void img_normalize(ImageMatrix* dst, const ImageMatrix src) {
    PIXEL_TYPE max_pixel = PIXEL(src, 0, 0);
    PIXEL_TYPE min_pixel = PIXEL(src, 0, 0);
    FOR_EACH_PIXEL(src, ) {
        max_pixel = MAX(max_pixel, PIXEL(src, row, col));
        min_pixel = MIN(min_pixel, PIXEL(src, row, col));
    }
    if (max_pixel == min_pixel) {
        return;
    }
    img_copy_dimensions(dst, src, 0);
    FOR_EACH_PIXEL(src, ) {
        PIXEL(*dst, row, col) =
                ((PIXEL(src, row, col) - min_pixel) * UINT8_MAX) / (max_pixel - min_pixel);
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

void img_draw_line(
        ImageMatrix mat, int16_t x0, int16_t y0, int16_t x1, int16_t y1, PIXEL_TYPE color) {
    // Bresenham's Line Algorithm
    uint8_t swap_xy = ABS(y1 - y0) > ABS(x1 - x0);
    if (swap_xy) {
        SWAP(x0, y0);
        SWAP(x1, y1);
    }
    if (x0 > x1) {
        SWAP(x0, x1);
        SWAP(y0, y1);
    }
    int16_t dx = x1 - x0;
    int16_t dy = y1 - y0;
    if (dy < 0) {
        dy = -dy;
        y0 = -y0;
    }
    int16_t error = dy * 2 - dx;
#define ITERATE_LINE(EDIT_PIXEL)                  \
    EDIT_PIXEL = color;                       \
    while (x0++ < x1) {                       \
        error += 2 * (dy - (error > 0) * dx); \
        y0 += error > 0;                      \
        EDIT_PIXEL = color;                   \
    }
    if (swap_xy) {
        ITERATE_LINE(PIXEL(mat, x0, ABS(y0)));
    } else {
        ITERATE_LINE(PIXEL(mat, ABS(y0), x0));
    }
}

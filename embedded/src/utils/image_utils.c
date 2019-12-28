#include "image_utils.h"
#include <assert.h>

uint8_t img_bilinear_interpolation(const ImageMatrix mat, Vector2f position) {
    int16_t right = position.x + 0.5f;
    int16_t bottom = position.y + 0.5f;
    int16_t left = MAX(right - 1, 0);
    int16_t top = MAX(bottom - 1, 0);
    right = MIN(right, mat.n_cols - 1);
    bottom = MIN(bottom, mat.n_rows - 1);
    Vector2f progress = {position.x - 0.5f - left, position.y - 0.5f - top};
    float top_average = (float) PIXEL(mat, top, left) +
                        progress.x * (PIXEL(mat, top, right) - PIXEL(mat, top, left));
    float bottom_average = (float) PIXEL(mat, bottom, left) +
                           progress.x * (PIXEL(mat, bottom, right) - PIXEL(mat, bottom, left));
    return top_average + progress.y * (bottom_average - top_average);
}

void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, uint8_t bg_fill,
        ImageInterpolation interpolation) {
    assert(interpolation);
    assert(!v2f_is_zero(rotation) && !v2f_is_nan(rotation));
    Vector2f src_center = {0.5f * src.n_cols, 0.5f * src.n_rows};
    Vector2f dst_center = {0.5f * dst.n_cols, 0.5f * dst.n_rows};
    rotation = v2f_flip_rotation(rotation);
    FOR_EACH_PIXEL(dst) {
        Vector2f from_center = {0.5f + col - dst_center.x, 0.5f + row - dst_center.y};
        Vector2f src_position = v2f_add(src_center, v2f_rotate(from_center, rotation));
        if (src_position.x < 0.0f || src_position.x >= src.n_cols || src_position.y < 0.0f ||
                src_position.y >= src.n_rows) {
            PIXEL(dst, row, col) = bg_fill;
            continue;
        }
        PIXEL(dst, row, col) = interpolation(src, src_position);
    }
}

void img_draw_line(ImageMatrix mat, ImageWindow line, uint8_t color) {
    // Bresenham's Line Algorithm
    uint8_t swap_xy = ABS(line.y1 - line.y0) > ABS(line.x1 - line.x0);
    if (swap_xy) {
        SWAP(line.x0, line.y0);
        SWAP(line.x1, line.y1);
    }
    if (line.x0 > line.x1) {
        SWAP(line.x0, line.x1);
        SWAP(line.y0, line.y1);
    }
    int16_t dx = line.x1 - line.x0;
    int16_t dy = line.y1 - line.y0;
    if (dy < 0) {
        dy = -dy;
        line.y0 = -line.y0;
    }
    int16_t error = dy * 2 - dx;
#define ITERATE_LINE(EDIT_PIXEL)              \
    EDIT_PIXEL = color;                       \
    while (line.x0++ < line.x1) {             \
        error += 2 * (dy - (error > 0) * dx); \
        line.y0 += error > 0;                 \
        EDIT_PIXEL = color;                   \
    }
    if (swap_xy) {
        ITERATE_LINE(PIXEL(mat, line.x0, ABS(line.y0)));
    } else {
        ITERATE_LINE(PIXEL(mat, ABS(line.y0), line.x0));
    }
}

void img_convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrixInt8 kernel) {
    dst->n_rows = src.n_rows - (kernel.n_rows - 1);
    dst->n_cols = src.n_cols - (kernel.n_cols - 1);
    FOR_EACH_PIXEL(*dst) {
        int32_t value = 0;
        IMG_APPLY_KERNEL(value, kernel, src, row, col);
        PIXEL(*dst, row, col) = CLAMP(value, 0, UINT8_MAX);
    }
}

void img_edge_filter(ImageMatrix* dst, const ImageMatrix src) {
    img_convolution(dst, src, edge_kernel);
}

void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src) {
    const uint8_t(*data_rgb888)[3] = (uint8_t(*)[3]) src.data;
    int32_t data_len = IMG_SIZE(src);
    for (int32_t i = 0; i < data_len; ++i) {
        dst->data[i] = (data_rgb888[i][0] + data_rgb888[i][1] + data_rgb888[i][2]) / 3;
    }
    IMG_COPY_SIZE(dst, src);
}

void img_hough_line_transform(ImageMatrixInt32 dst, const ImageMatrix src) {
    IMG_FILL(dst, 0);
    float angle_resolution = M_PI / dst.n_rows;
    float scale_to_index =
            dst.n_cols / sqrtf((src.n_rows * src.n_rows) + (src.n_cols * src.n_cols));
    for (int16_t i = 0; i < dst.n_rows; ++i) {
        float sin = sinf(i * angle_resolution);
        float cos = cosf(i * angle_resolution);
        FOR_EACH_PIXEL(src) {
            PIXEL(dst, i, (int16_t)(((sin * row) + (cos * col)) * scale_to_index)) +=
                    PIXEL(src, row, col);
        }
    }
    IMG_NORMALIZE(&dst, dst);
}

#include "image_utils.h"
#include <assert.h>

uint8_t img_average(const ImageMatrix mat) {
    uint32_t sum = 0;
    FOR_EACH_PIXEL(mat) { sum += PIXEL(mat, row, col); }
    return sum / (mat.n_rows * mat.n_cols);
}

void img_threshold(ImageMatrix* dst, const ImageMatrix src, uint8_t threshold) {
    IMG_COPY_SIZE(dst, src);
    FOR_EACH_PIXEL(src) { PIXEL(*dst, row, col) = (PIXEL(src, row, col) >= threshold) * UINT8_MAX; }
}

void img_normalize(ImageMatrix* dst, const ImageMatrix src) {
    uint8_t max_pixel = PIXEL(src, 0, 0);
    uint8_t min_pixel = PIXEL(src, 0, 0);
    FOR_EACH_PIXEL(src) {
        max_pixel = MAX(max_pixel, PIXEL(src, row, col));
        min_pixel = MIN(min_pixel, PIXEL(src, row, col));
    }
    if (max_pixel == min_pixel) {
        return;
    }
    IMG_COPY_SIZE(dst, src);
    FOR_EACH_PIXEL(src) {
        PIXEL(*dst, row, col) =
                ((PIXEL(src, row, col) - min_pixel) * UINT8_MAX) / (max_pixel - min_pixel);
    }
}

void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, uint8_t bg_fill) {
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

void img_draw_line(ImageMatrix mat, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color) {
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
#define ITERATE_LINE(EDIT_PIXEL)              \
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

void img_convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    dst->n_rows = src.n_rows - (kernel.n_rows - 1);
    dst->n_cols = src.n_cols - (kernel.n_cols - 1);
    FOR_EACH_PIXEL(*dst) {
        int32_t value = 0;
        IMG_APPLY_KERNEL(value, kernel, src, row, col);
        PIXEL(*dst, row, col) = CLAMP(value, 0, UINT8_MAX);
    }
}

void img_edge_filter(ImageMatrix* dst, const ImageMatrix src) {
    img_convolution(dst, src, edge_detect_kernel);
}

void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src) {
    const uint8_t(*data_rgb888)[3] = (uint8_t(*)[3]) src.data;
    int32_t data_len = src.n_rows * src.n_cols;
    for (int32_t i = 0; i < data_len; ++i) {
        dst->data[i] = (data_rgb888[i][0] + data_rgb888[i][1] + data_rgb888[i][2]) / 3;
    }
    IMG_COPY_SIZE(dst, src);
}

uint32_t img_count_negative(ImageMatrixFloat mat) {
    uint32_t count = 0;
    FOR_EACH_PIXEL(mat) { count += PIXEL(mat, row, col) < 0; }
    return count;
}

void img_hough_line_transform(ImageMatrixFloat dst, const ImageMatrix src) {
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
    assert(img_count_negative(dst) == 0);
}

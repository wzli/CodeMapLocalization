#include "image_utils.h"
#include <assert.h>

void img_convolution_filter(ImageMatrix* dst, const ImageMatrix src, const ImageMatrixInt8 kernel) {
    IMG_CONVOLUTION(dst, src, kernel, 1, 0, UINT8_MAX);
}

void img_max_filter(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    IMG_REDUCE_FILTER(dst, src, kernel, 0, MAX);
}

void img_min_filter(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    IMG_REDUCE_FILTER(dst, src, kernel, UINT8_MAX, MIN);
}

void img_median_filter(ImageMatrix* dst, const ImageMatrix src, ImageMatrix window) {
    assert(window.data);
    assert(IMG_SIZE(window) > 1);
    IMG_VALID_PADDING(dst, src, window);
    int16_t middle_index = IMG_SIZE(window) / 2;
    FOR_EACH_PIXEL(*dst) {
        ImageWindow crop_area = {col, row, col + window.n_cols, row + window.n_rows};
        IMG_CROP(&window, src, crop_area);
        QUICK_SELECT(window.data, IMG_SIZE(window), middle_index);
        PIXEL(*dst, row, col) = window.data[middle_index];
    }
}

uint8_t img_nearest_interpolation(const ImageMatrix mat, Vector2f position) {
    return PIXEL(mat, CLAMP((int16_t) position.y, 0, mat.n_rows - 1),
            CLAMP((int16_t) position.x, 0, mat.n_cols - 1));
}

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

static inline float cubic_interpolation(float p[4], float x) {
    return p[1] + 0.5f * x *
                          (p[2] - p[0] +
                                  x * (2.0f * p[0] - 5.0f * p[1] + 4.0f * p[2] - p[3] +
                                              x * (3.0f * (p[1] - p[2]) + p[3] - p[0])));
}

uint8_t img_bicubic_interpolation(const ImageMatrix mat, Vector2f position) {
    int16_t row0 = position.y - 1.5f;
    int16_t col0 = position.x - 1.5f;
    float y_points[4];
    for (int16_t i = 0; i < 4; ++i) {
        int16_t row = CLAMP(row0 + i, 0, mat.n_rows - 1);
        float x_points[4];
        for (int16_t j = 0; j < 4; ++j) {
            int16_t col = CLAMP(col0 + j, 0, mat.n_cols - 1);
            x_points[j] = PIXEL(mat, row, col);
        }
        y_points[i] = cubic_interpolation(x_points, position.x - col0 - 1.5f);
    }
    float result = cubic_interpolation(y_points, position.y - row0 - 1.5f);
    return CLAMP(result, 0, UINT8_MAX);
}

void img_resize(ImageMatrix dst, const ImageMatrix src, ImageInterpolation interpolation) {
    assert(interpolation);
    Vector2f scale = {(float) src.n_cols / dst.n_cols, (float) src.n_rows / dst.n_rows};
    FOR_EACH_PIXEL(dst) {
        Vector2f position = {0.5f + col, 0.5f + row};
        PIXEL(dst, row, col) = interpolation(src, v2f_multiply(position, scale));
    }
}

void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, uint8_t bg_fill,
        ImageInterpolation interpolation) {
    Matrix2f transform = {{rotation.x, -rotation.y, rotation.y, rotation.x}};
    img_affine_transform(dst, src, transform, bg_fill, interpolation);
}

void img_affine_transform(ImageMatrix dst, const ImageMatrix src, Matrix2f transform,
        uint8_t bg_fill, ImageInterpolation interpolation) {
    assert(interpolation);
    assert(!m2f_is_nan(transform));
    assert(m2f_determinant(transform) != 0.0f);
    Vector2f src_center = {0.5f * src.n_cols, 0.5f * src.n_rows};
    Vector2f dst_center = {0.5f * dst.n_cols, 0.5f * dst.n_rows};
    transform = m2f_inverse(transform);
    FOR_EACH_PIXEL(dst) {
        Vector2f from_center = {0.5f + col - dst_center.x, 0.5f + row - dst_center.y};
        Vector2f src_position = v2f_add(src_center, m2f_transform(transform, from_center));
        if (src_position.x < 0.0f || src_position.x >= src.n_cols || src_position.y < 0.0f ||
                src_position.y >= src.n_rows) {
            PIXEL(dst, row, col) = bg_fill;
            continue;
        }
        PIXEL(dst, row, col) = interpolation(src, src_position);
    }
}

void img_histogram(uint32_t histogram[256], const ImageMatrix mat) {
    for (uint16_t i = 0; i < 256; ++i) {
        histogram[i] = 0;
    }
    FOR_EACH_PIXEL(mat) { ++histogram[PIXEL(mat, row, col)]; }
}

uint8_t img_compute_otsu_threshold(const uint32_t histogram[256]) {
    // "A C++ Implementation of Otsu’s Image Segmentation Method", 2016.
    int32_t N = 0;
    int32_t sum = 0;
    for (int16_t i = 0; i < 256; ++i) {
        N += histogram[i];
        sum += i * histogram[i];
    }
    uint8_t threshold = 0;
    int32_t sum_b = 0;
    int32_t q1 = 0;
    float max_variance = 0;
    for (int16_t i = 0; i < 256; ++i) {
        q1 += histogram[i];
        if (!q1) {
            continue;
        }
        int32_t q2 = N - q1;
        if (!q2) {
            break;
        }
        sum_b += i * histogram[i];
        float mean_difference = (float) sum_b / q1 - (float) (sum - sum_b) / q2;
        float variance = SQR(mean_difference) * q1 * q2;
        if (variance >= max_variance) {
            threshold = i;
            max_variance = variance;
        }
    }
    return threshold;
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

void img_draw_rectangle(ImageMatrix mat, ImageWindow rect, uint8_t color) {
    for (int16_t row = rect.y0; row <= rect.y1; ++row) {
        PIXEL(mat, row, rect.x0) = color;
        PIXEL(mat, row, rect.x1) = color;
    }
    for (int16_t col = rect.x0; col <= rect.x1; ++col) {
        PIXEL(mat, rect.y0, col) = color;
        PIXEL(mat, rect.y1, col) = color;
    }
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

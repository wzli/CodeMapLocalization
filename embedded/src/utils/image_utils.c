#include "image_utils.h"
#include <assert.h>

void img_convolution_filter(ImageMatrix* dst, const ImageMatrix src, const ImageMatrixInt8 kernel) {
    IMG_CONVOLUTION(*dst, src, kernel, 1, 0, UINT8_MAX);
}

void img_max_filter(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    IMG_REDUCE_FILTER(*dst, src, kernel, 0, MAX);
}

void img_min_filter(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    IMG_REDUCE_FILTER(*dst, src, kernel, UINT8_MAX, MIN);
}

void img_median_filter(ImageMatrix* dst, const ImageMatrix src, ImageMatrix window) {
    assert(window.data);
    assert(IMG_SIZE(window) > 1);
    IMG_VALID_PADDING(*dst, src, window);
    int16_t middle_index = IMG_SIZE(window) / 2;
    FOR_EACH_PIXEL(*dst) {
        ImagePoint top_left = {col, row};
        IMG_CROP(window, src, top_left);
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

void img_draw_line(ImageMatrix mat, ImagePoint from, ImagePoint to, uint8_t color, uint8_t width) {
    // Bresenham's Line Algorithm
    assert(width);
    int16_t dx = to.x - from.x;
    int16_t dy = to.y - from.y;
    uint8_t swap_xy = ABS(dy) > ABS(dx);
    if (swap_xy) {
        SWAP(from.x, from.y);
        SWAP(to.x, to.y);
    }
    if (from.x > to.x) {
        SWAP(from.x, to.x);
        SWAP(from.y, to.y);
    }
    dx = to.x - from.x;
    dy = to.y - from.y;
    int8_t dir = 1;
    if (dy < 0) {
        dy = -dy;
        dir = -1;
    }
    float slope = (float) dy / dx;
    width *= sqrtf(1.0f + SQR(slope));
    int16_t error = dy * 2 - dx;
#define ITERATE_LINE(X, Y)                                                       \
    for (int16_t y = from.y, x = from.x; x <= to.x;                              \
            ++x, error += 2 * (dy - (error > 0) * dx), y += dir * (error > 0)) { \
        from.y = y - (width - 1) / 2;                                            \
        from.y = MAX(from.y, 0);                                                 \
        from.x = MAX(x, 0);                                                      \
        to.y = y + width / 2;                                                    \
        to.Y = MIN(to.Y, mat.n_rows - 1);                                        \
        to.X = MIN(to.X, mat.n_cols - 1);                                        \
        for (; from.y <= to.y; ++from.y) {                                       \
            PIXEL(mat, from.Y, from.X) = color;                                  \
        }                                                                        \
    }
    if (swap_xy) {
        ITERATE_LINE(y, x);
    } else {
        ITERATE_LINE(x, y);
    }
}

void img_draw_box(ImageMatrix mat, ImagePoint from, ImagePoint to, uint8_t color, uint8_t width) {
    assert(width);
    if (from.x > to.x) {
        SWAP(from.x, to.x);
    }
    if (from.y > to.y) {
        SWAP(from.y, to.y);
    }
    from.x = MAX(from.x, 0);
    from.y = MAX(from.y, 0);
    to.x = MIN(to.x, mat.n_cols - 1);
    to.y = MIN(to.y, mat.n_rows - 1);
    int16_t dx = to.x - from.x;
    int16_t dy = to.y - from.y;
    width = MIN(width, dx);
    width = MIN(width, dy);
    for (int16_t row = from.y; row <= to.y; ++row) {
        for (uint8_t i = 0; i < width; ++i) {
            PIXEL(mat, row, from.x + i) = color;
            PIXEL(mat, row, to.x - i) = color;
        }
    }
    for (int16_t col = from.x; col <= to.x; ++col) {
        for (uint8_t i = 0; i < width; ++i) {
            PIXEL(mat, from.y + i, col) = color;
            PIXEL(mat, to.y - i, col) = color;
        }
    }
}

void img_draw_polygon(
        ImageMatrix mat, const ImagePoint* vertices, uint8_t len, uint8_t color, uint8_t width) {
    for (uint8_t prev = 0; len--; prev = len) {
        img_draw_line(mat, vertices[prev], vertices[len], color, width);
    }
}

void img_draw_regular_polygon(ImageMatrix mat, ImagePoint center, Vector2f center_to_vertex,
        uint8_t order, uint8_t color, uint8_t width) {
    Vector2f rotation_increment = {cosf(2 * (float) M_PI / order), sinf(2 * (float) M_PI / order)};
    ImagePoint previous_vertex = {center.x + center_to_vertex.x, center.y + center_to_vertex.y};
    for (uint8_t i = 0; i < order; ++i) {
        center_to_vertex = v2f_rotate(center_to_vertex, rotation_increment);
        ImagePoint next_vertex = {center.x + center_to_vertex.x, center.y + center_to_vertex.y};
        img_draw_line(mat, previous_vertex, next_vertex, color, width);
        previous_vertex = next_vertex;
    };
}

void img_hough_line_transform(ImageMatrixInt32 dst, const ImageMatrix src) {
    IMG_FILL(dst, 0);
    float angle_resolution = M_PI / dst.n_rows;
    float scale_to_index =
            dst.n_cols / sqrtf((src.n_rows * src.n_rows) + (src.n_cols * src.n_cols));
    const Vector2f rot_inc = {cosf(angle_resolution), sinf(angle_resolution)};
    Vector2f rot = {1, 0};
    for (int16_t i = 0; i < dst.n_rows; ++i) {
        rot = v2f_rotate(rot, rot_inc);
        FOR_EACH_PIXEL(src) {
            PIXEL(dst, i, (int16_t)(((rot.y * row) + (rot.x * col)) * scale_to_index)) +=
                    PIXEL(src, row, col);
        }
    }
    IMG_NORMALIZE(dst, dst);
}

#define L1_DISTANCE_TRANSFORM_1D(DST, SRC, LEN, STRIDE)                                    \
    do {                                                                                   \
        (DST)[0] = (SRC)[0];                                                               \
        for (int16_t I = 1; I < (LEN); ++I) {                                              \
            (DST)[I * (STRIDE)] = MIN((DST)[(I - 1) * (STRIDE)] + 1, (SRC)[I * (STRIDE)]); \
        }                                                                                  \
        for (int16_t I = (LEN) -2; I >= 0; --I) {                                          \
            (DST)[I * (STRIDE)] = MIN((DST)[(I + 1) * (STRIDE)] + 1, (DST)[I * (STRIDE)]); \
        }                                                                                  \
    } while (0)

#define SQUARE_DISTANCE_TRANSFORM_1D(DST, SRC, LEN, STRIDE)           \
    do {                                                              \
        int16_t x1 = 0;                                               \
        int32_t y1 = (SRC)[0];                                        \
        (DST)[0] = y1;                                                \
        for (int16_t x2 = 1; x2 < (LEN); ++x2) {                      \
            int16_t dx = x2 - x1;                                     \
            int32_t dy = (SRC)[x2 * (STRIDE)] - (DST)[x1 * (STRIDE)]; \
            int32_t intersection = dy + dx * (x2 + x1);               \
            if (2 * dx * x2 >= intersection) {                        \
                x1 = x2;                                              \
                y1 = (SRC)[x2 * (STRIDE)];                            \
                (DST)[x2 * (STRIDE)] = y1;                            \
                for (dx = 1; dx <= x2; ++dx) {                        \
                    int32_t y = SQR(dx) + y1;                         \
                    if (y < (DST)[(x2 - dx) * (STRIDE)]) {            \
                        (DST)[(x2 - dx) * (STRIDE)] = y;              \
                    } else {                                          \
                        break;                                        \
                    }                                                 \
                }                                                     \
            } else {                                                  \
                (DST)[x2 * (STRIDE)] = y1 + SQR(dx);                  \
                if (2 * (dx + 1) * (x2 + 1) >= intersection) {        \
                    x1 = x2;                                          \
                    y1 = (SRC)[x2 * (STRIDE)];                        \
                }                                                     \
            }                                                         \
        }                                                             \
    } while (0)

#define SEPARABLE_2D_TRANSFORM(DST, SRC, TRANSFORM_1D)                                    \
    do {                                                                                  \
        IMG_COPY_SIZE(DST, SRC);                                                          \
        for (int16_t row = 0; row < (SRC).n_rows; ++row, (DST).data += (SRC).n_cols) {    \
            TRANSFORM_1D((DST).data, (SRC).data + row * (SRC).n_cols, (SRC).n_cols, 1);   \
        }                                                                                 \
        (DST).data -= IMG_SIZE(DST);                                                      \
        for (int16_t col = 0; col < (SRC).n_cols; ++col) {                                \
            TRANSFORM_1D((DST).data + col, (DST).data + col, (SRC).n_rows, (SRC).n_cols); \
        }                                                                                 \
    } while (0)

void img_l1_distance_transform(ImageMatrixInt32 dst, const ImageMatrix src) {
    SEPARABLE_2D_TRANSFORM(dst, src, L1_DISTANCE_TRANSFORM_1D);
}

void img_square_distance_transform(ImageMatrixInt32 dst, const ImageMatrix src) {
    SEPARABLE_2D_TRANSFORM(dst, src, SQUARE_DISTANCE_TRANSFORM_1D);
}

void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src) {
    const uint8_t(*data_rgb888)[3] = (uint8_t(*)[3]) src.data;
    int32_t data_len = IMG_SIZE(src);
    for (int32_t i = 0; i < data_len; ++i) {
        dst->data[i] = (data_rgb888[i][0] + data_rgb888[i][1] + data_rgb888[i][2]) / 3;
    }
    IMG_COPY_SIZE(*dst, src);
}

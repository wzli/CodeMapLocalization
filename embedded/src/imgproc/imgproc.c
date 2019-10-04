#include "imgproc.h"
#include "bit_matrix/bit_matrix.h"

#include <limits.h>
#include <float.h>
#include <math.h>
#include <assert.h>

#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define ELEMENT(MATRIX, ROW, COL) \
    ((MATRIX).data[(ROW) * (MATRIX).n_cols + (COL)])

#define FOR_EACH_ELEMENT(MAT)                        \
    for (int16_t row = 0; row < (MAT).n_rows; ++row) \
        for (int16_t col = 0; col < (MAT).n_cols; ++col)

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define SQR(x) ((x) * (x))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))

static int16_t buf_data[128 * 128];
static int16_t sobel_kernel_x[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
static int16_t sobel_kernel_y[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};

void print_bits(uint32_t word, int8_t word_length) {
    for (word_length--; word_length >= 0; word_length--) {
        printf("%u", (word >> word_length) & 1);
    }
    puts("");
}

void print_matrix(ImageMatrix src) {
    for (int16_t row = 0; row < src.n_rows; ++row) {
        for (int16_t col = 0; col < src.n_cols; ++col) {
            printf("%6d ", ELEMENT(src, row, col));
        }
        puts("");
    }
    puts("");
}

int32_t count_negative_elements(ImageMatrix mat) {
    int32_t count = 0;
    FOR_EACH_ELEMENT(mat) { count += ELEMENT(mat, row, col) < 0; }
    return count;
}

void threshold_elements(ImageMatrix mat, int16_t threshold, int16_t max_val) {
    FOR_EACH_ELEMENT(mat) {
        ELEMENT(mat, row, col) =
                (ELEMENT(mat, row, col) >= threshold) * max_val;
    }
}

void normalize_elements(ImageMatrix mat, int16_t max_val) {
    int16_t max_element = SHRT_MIN;
    int16_t min_element = SHRT_MAX;
    FOR_EACH_ELEMENT(mat) {
        max_element = MAX(max_element, ELEMENT(mat, row, col));
        min_element = MIN(min_element, ELEMENT(mat, row, col));
    }
    if (max_element == min_element) {
        return;
    }
    FOR_EACH_ELEMENT(mat) {
        ELEMENT(mat, row, col) =
                ((ELEMENT(mat, row, col) - min_element) * max_val) /
                (max_element - min_element);
    }
};

void convolution(
        ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    dst->n_rows = src.n_rows - (kernel.n_rows & ~1);
    dst->n_cols = src.n_cols - (kernel.n_cols & ~1);
    FOR_EACH_ELEMENT(*dst) { ELEMENT(*dst, row, col) = 0; }
    for (int16_t k_row = 0; k_row < kernel.n_rows; ++k_row) {
        for (int16_t k_col = 0; k_col < kernel.n_cols; ++k_col) {
            FOR_EACH_ELEMENT(*dst) {
                ELEMENT(*dst, row, col) +=
                        ELEMENT(kernel, k_row, k_col) *
                        ELEMENT(src, row + k_row, col + k_col);
            }
        }
    }
}

inline float vector2f_dot(Vector2f a, Vector2f b) {
    return a.x * b.x + a.y * b.y;
}

inline float vector2f_distance_squared(Vector2f a, Vector2f b) {
    return SQR(a.x - b.x) + SQR(a.y - b.y);
}

inline Vector2f vector2f_midpoint(Vector2f a, Vector2f b) {
    return (Vector2f){0.5f * (a.x + b.x), 0.5f * (a.y + b.y)};
}

inline Vector2f vector2f_rotate(Vector2f vec, Vector2f rot) {
    return (Vector2f){(rot.x * vec.x) - (rot.y * vec.y),
            (rot.y * vec.x) + (rot.x * vec.y)};
}

inline Vector2f vector2f_normalize(Vector2f vec) {
    float norm_scale = 1.0f / sqrt(SQR(vec.x) + SQR(vec.y));
    return (Vector2f){vec.x * norm_scale, vec.y * norm_scale};
}

inline Vector2f vector2f_double_angle(Vector2f rot) {
    return (Vector2f){SQR(rot.x) - SQR(rot.y), 2.0f * rot.x * rot.y};
}

inline Vector2f vector2f_half_angle(Vector2f rot) {
    return (Vector2f){sqrtf((1.0f + rot.x) * 0.5f),
            copysignf(sqrtf((1.0f - rot.x) * 0.5f), rot.y)};
}

Vector2f estimate_rotation(const ImageMatrix mat) {
    Vector2f gradient_sum = {};
    int16_t n_rows_bound = mat.n_rows - 2;
    int16_t n_cols_bound = mat.n_cols - 2;
    for (int16_t mat_row = 0; mat_row < n_rows_bound; mat_row++) {
        for (int16_t mat_col = 0; mat_col < n_cols_bound; mat_col++) {
            Vector2f gradient = {};
            for (int16_t k_row = 0; k_row < 3; k_row++) {
                for (int16_t k_col = 0; k_col < 3; k_col++) {
                    gradient.x +=
                            sobel_kernel_x[(k_row * 3) + k_col] *
                            ELEMENT(mat, k_row + mat_row, k_col + mat_col);
                    gradient.y +=
                            sobel_kernel_y[(k_row * 3) + k_col] *
                            ELEMENT(mat, k_row + mat_row, k_col + mat_col);
                }
            }
            gradient = vector2f_double_angle(gradient);
            gradient = vector2f_double_angle(gradient);
            gradient_sum.x += gradient.x;
            gradient_sum.y += gradient.y;
        }
    }
    if (gradient_sum.x != 0 || gradient_sum.y != 0) {
        gradient_sum = vector2f_normalize(gradient_sum);
        gradient_sum = vector2f_half_angle(gradient_sum);
        gradient_sum = vector2f_half_angle(gradient_sum);
        assert(!isnan(gradient_sum.x) && !isnan(gradient_sum.y));
    }
    return gradient_sum;
}

void rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation) {
    assert(rotation.x != 0 || rotation.y != 0);
    assert(!isnan(rotation.x) && !isnan(rotation.y));
    Vector2f src_center = {0.5f * src.n_cols, 0.5f * src.n_rows};
    Vector2f dst_center = {0.5f * dst.n_cols, 0.5f * dst.n_rows};
    FOR_EACH_ELEMENT(dst) {
        Vector2f from_center = {
                0.5f + col - dst_center.x, 0.5f + row - dst_center.y};
        Vector2f src_position = {(rotation.x * from_center.x) -
                                         (rotation.y * from_center.y) +
                                         src_center.x,
                (rotation.y * from_center.x) + (rotation.x * from_center.y) +
                        src_center.y};
        if (src_position.x < 0.0f || src_position.x >= src.n_cols ||
                src_position.y < 0.0f || src_position.y >= src.n_rows) {
            continue;
        }
        int16_t right = round(src_position.x);
        int16_t bottom = round(src_position.y);
        int16_t left = MAX(right - 1, 0);
        int16_t top = MAX(bottom - 1, 0);
        right = MIN(right, src.n_cols - 1);
        bottom = MIN(bottom, src.n_rows - 1);
        Vector2f progress = {
                src_position.x - 0.5f - left, src_position.y - 0.5f - top};
        float top_average = (float) ELEMENT(src, top, left) +
                            progress.x * (ELEMENT(src, top, right) -
                                                 ELEMENT(src, top, left));
        float bottom_average = (float) ELEMENT(src, bottom, left) +
                               progress.x * (ELEMENT(src, bottom, right) -
                                                    ELEMENT(src, bottom, left));
        ELEMENT(dst, row, col) =
                top_average + progress.y * (bottom_average - top_average);
    }
}

void rotation_bit_mask(
        BitMatrix32 rotated_mask, const ImageMatrix src, Vector2f rotation) {
    assert(rotation.x != 0 || rotation.y != 0);
    assert(!isnan(rotation.x) && !isnan(rotation.y));
    if (rotation.x < 0) {
        rotation.x = -rotation.x;
        rotation.y = -rotation.y;
    }
    rotation.x += FLT_EPSILON;
    rotation.y += (rotation.y == 0) * FLT_EPSILON;
    Vector2f rotated_corners[2];
    for (uint8_t i = 0; i < 2; ++i) {
        Vector2f from_center = {
                (i - 0.5f) * (src.n_cols - 1), (i - 0.5f) * (src.n_rows - 1)};
        rotated_corners[i].x =
                rotation.x * from_center.x - rotation.y * from_center.y + 16;
        rotated_corners[i].y =
                rotation.y * from_center.x + rotation.x * from_center.y + 16;
    }
    float inv_slope = rotation.x / rotation.y;
    float inv_slope_90 = -rotation.y / rotation.x;
    float x_intercepts[4] = {
            rotated_corners[0].x - inv_slope * (rotated_corners[0].y - 0.5f),
            rotated_corners[1].x - inv_slope_90 * (rotated_corners[1].y + 0.5f -
                                                          (rotation.y < 0)),
            rotated_corners[1].x - inv_slope * (rotated_corners[1].y + 0.5f),
            rotated_corners[0].x - inv_slope_90 * (rotated_corners[0].y - 0.5f +
                                                          (rotation.y < 0)),
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
        rotated_mask[i] = range_end <= range_begin
                                  ? 0u
                                  : (~0u << (uint8_t) range_begin) &
                                            (~0u >> (31 - (uint8_t) range_end));
    }
}

void edge_filter(ImageMatrix* dst, const ImageMatrix src) {
    // compute x gradient
    ImageMatrix buf_mat = {buf_data, 0, 0};
    ImageMatrix kernel = {sobel_kernel_x, 3, 3};
    convolution(&buf_mat, src, kernel);
    // compute y gradient
    kernel.data = sobel_kernel_y;
    convolution(dst, src, kernel);
    // sum absolute value of gradient components
    FOR_EACH_ELEMENT(buf_mat) {
        ELEMENT(*dst, row, col) = (SQR(ELEMENT(*dst, row, col)) +
                                          SQR(ELEMENT(buf_mat, row, col))) >>
                                  6;
    }
    assert(count_negative_elements(*dst) == 0);
}

void hough_line_transform(ImageMatrix dst, const ImageMatrix src) {
    FOR_EACH_ELEMENT(dst) { ELEMENT(dst, row, col) = 0; }
    float angle_resolution = M_PI * 0.5f / dst.n_rows;
    float scale_to_index = dst.n_cols / sqrtf((src.n_rows * src.n_rows) +
                                                (src.n_cols * src.n_cols));
    for (uint16_t i = 0; i < dst.n_rows; ++i) {
        float sin = sinf(i * angle_resolution);
        float cos = cosf(i * angle_resolution);
        FOR_EACH_ELEMENT(src) {
            ELEMENT(dst, i,
                    (int16_t)(((sin * row) + (cos * col)) * scale_to_index)) +=
                    ELEMENT(src, row, col);
        }
    }
    assert(count_negative_elements(dst) == 0);
};

void convert_uint8_to_int16(ImageMatrix mat) {
    uint8_t* data_uint8 = (uint8_t*) mat.data;
    for (int32_t i = mat.n_rows * mat.n_cols - 1; i >= 0; --i) {
        mat.data[i] = data_uint8[i];
    }
};

void convert_int16_to_uint8(ImageMatrix mat) {
    uint8_t* data_uint8 = (uint8_t*) mat.data;
    int32_t data_len = mat.n_rows * mat.n_cols;
    for (int32_t i = 0; i < data_len; ++i) {
        data_uint8[i] = mat.data[i];
    }
};

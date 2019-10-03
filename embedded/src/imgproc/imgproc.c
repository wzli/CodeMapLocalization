#include "imgproc.h"

#include <limits.h>
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
#define SQUARE(x) ((x) * (x))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))

static int16_t buf_data[128 * 128];
static int16_t sobel_kernel_x[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
static int16_t sobel_kernel_y[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};

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
    return SQUARE(a.x - b.x) + SQUARE(a.y - b.y);
}

inline Vector2f vector2f_midpoint(Vector2f a, Vector2f b) {
    return (Vector2f){0.5f * (a.x + b.x), 0.5f * (a.y + b.y)};
}

inline Vector2f vector2f_rotate(Vector2f vec, Vector2f rot) {
    return (Vector2f){(rot.x * vec.x) - (rot.y * vec.y),
            (rot.y * vec.x) + (rot.x * vec.y)};
}

inline Vector2f vector2f_normalize(Vector2f vec) {
    float norm_scale = 1.0f / sqrt(SQUARE(vec.x) + SQUARE(vec.y));
    return (Vector2f){vec.x * norm_scale, vec.y * norm_scale};
}

inline Vector2f vector2f_double_angle(Vector2f rot) {
    return (Vector2f){SQUARE(rot.x) - SQUARE(rot.y), 2.0f * rot.x * rot.y};
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
    float src_mid_row = 0.5f * src.n_rows;
    float src_mid_col = 0.5f * src.n_cols;
    float dst_mid_row = 0.5f * dst.n_rows;
    float dst_mid_col = 0.5f * dst.n_cols;
    FOR_EACH_ELEMENT(dst) {
        float x_from_center = 0.5f + col - dst_mid_col;
        float y_from_center = 0.5f + row - dst_mid_row;
        Vector2f src_position = {(rotation.x * x_from_center) +
                                         (rotation.y * y_from_center) +
                                         src_mid_col,
                -(rotation.y * x_from_center) + (rotation.x * y_from_center) +
                        src_mid_row};
        if (src_position.x < 0.0f || src_position.x >= src.n_cols ||
                src_position.y < 0.0f || src_position.y >= src.n_rows) {
            continue;
        }
        int16_t right = round(src_position.x);
        int16_t bottom = round(src_position.y);
        float horizontal_progress = 0.5f - src_position.x + right;
        float vertical_progress = 0.5f - src_position.y + bottom;
        if (right >= src.n_cols) {
            if (bottom >= src.n_rows) {
                ELEMENT(dst, row, col) = ELEMENT(src, bottom - 1, right - 1);
            } else if (bottom <= 0) {
                ELEMENT(dst, row, col) = ELEMENT(src, bottom, right - 1);
            } else {
                ELEMENT(dst, row, col) =
                        (float) ELEMENT(src, bottom, right - 1) +
                        vertical_progress *
                                (ELEMENT(src, bottom - 1, right - 1) -
                                        ELEMENT(src, bottom, right - 1));
            }
        } else if (right <= 0) {
            if (bottom >= src.n_rows) {
                ELEMENT(dst, row, col) = ELEMENT(src, bottom - 1, right);
            } else if (bottom <= 0) {
                ELEMENT(dst, row, col) = ELEMENT(src, bottom, right);
            } else {
                ELEMENT(dst, row, col) =
                        (float) ELEMENT(src, bottom, right) +
                        vertical_progress *
                                (ELEMENT(src, bottom - 1, right) -
                                        ELEMENT(src, bottom, right));
            }
        } else if (bottom >= src.n_rows) {
            ELEMENT(dst, row, col) =
                    (float) ELEMENT(src, bottom - 1, right) +
                    horizontal_progress *
                            (ELEMENT(src, bottom - 1, right - 1) -
                                    ELEMENT(src, bottom - 1, right));
        } else if (bottom <= 0) {
            ELEMENT(dst, row, col) =
                    (float) ELEMENT(src, bottom, right) +
                    horizontal_progress * (ELEMENT(src, bottom, right - 1) -
                                                  ELEMENT(src, bottom, right));
        } else {
            float top_average =
                    (float) ELEMENT(src, bottom - 1, right) +
                    horizontal_progress *
                            (ELEMENT(src, bottom - 1, right - 1) -
                                    ELEMENT(src, bottom - 1, right));
            float bottom_average =
                    (float) ELEMENT(src, bottom, right) +
                    horizontal_progress * (ELEMENT(src, bottom, right - 1) -
                                                  ELEMENT(src, bottom, right));
            ELEMENT(dst, row, col) =
                    bottom_average +
                    vertical_progress * (top_average - bottom_average);
        }
        // ELEMENT(dst, row, col) = ELEMENT(src, (uint16_t)src_position.y,
        // (uint16_t)src_position.x);
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
        ELEMENT(*dst, row, col) = (SQUARE(ELEMENT(*dst, row, col)) +
                                          SQUARE(ELEMENT(buf_mat, row, col))) >>
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

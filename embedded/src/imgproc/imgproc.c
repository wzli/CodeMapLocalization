#include "imgproc.h"

#include <limits.h>
#include <math.h>
#include <assert.h>

#include <stdio.h>

#define ELEMENT(MATRIX, ROW, COL) \
    ((MATRIX).data[(ROW) * (MATRIX).n_cols + (COL)])

#define FOR_EACH_ELEMENT(MAT)                        \
    for (int16_t row = 0; row < (MAT).n_rows; ++row) \
        for (int16_t col = 0; col < (MAT).n_cols; ++col)

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define SQUARE(x) ((x) * (x))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void print_matrix(ImageMatrix src) {
    for (int16_t row = 0; row < src.n_rows; ++row) {
        for (int16_t col = 0; col < src.n_cols; ++col) {
            printf("%6d ", ELEMENT(src, row, col));
        }
        puts("");
    }
    puts("");
}

static int16_t buf_data[128 * 128];
static int16_t sobel_kernel_x[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
static int16_t sobel_kernel_y[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};

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

static const Vector2 ROTATION_VECTORS[] = {
    {1 , 0},
    {2 , 1},
    {1 , 1},
    {1 , 2},
    };

#define N_ROTATION_VECTORS (sizeof(ROTATION_VECTORS) / sizeof(Vector2))

float dot_product(Vector2 a, Vector2 b) {
    return (float)a.x * b.x + (float)a.y * b.y;
}

int32_t squared_distance(Vector2 a, Vector2 b) {
    float da = dot_product(a,a);
    float db = dot_product(b,b);
    a.x *= 10000000 / sqrtf(da);
    a.y *= 10000000 / sqrtf(da);
    b.x *= 10000000 / sqrtf(db);
    b.y *= 10000000 / sqrtf(db);
    return (SQUARE((a.x-b.x)>>12)) + (SQUARE((a.y-b.y)>>12));
}


float cosine(Vector2 a, Vector2 b) {
    return dot_product(a, b)/(sqrtf(dot_product(a,a)) * sqrtf(dot_product(b,b)));
}

Vector2 rotate_vector(Vector2 vector, Vector2 rotation) {
    return (Vector2){
        (rotation.x * vector.x) - (rotation.y * vector.y),
        (rotation.y * vector.x) + (rotation.x * vector.y)
    };
}

Vector2 estimate_rotation(const ImageMatrix mat) {
    Vector2 rotation_estimates[N_ROTATION_VECTORS] = {};
    int16_t n_rows_bound = mat.n_rows - 2;
    int16_t n_cols_bound = mat.n_cols - 2;
    for (int16_t mat_row = 0; mat_row < n_rows_bound; mat_row++) {
        for (int16_t mat_col = 0; mat_col < n_cols_bound; mat_col++) {
            Vector2 gradient = {};
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
            for(int16_t i = 0; i < N_ROTATION_VECTORS; ++i) {
                Vector2 rotated_gradient = rotate_vector(gradient, ROTATION_VECTORS[i]);
                if (rotated_gradient.x == 0 || rotated_gradient.y == 0) {
                    rotation_estimates[i].x += ABS(rotated_gradient.x) + ABS(rotated_gradient.y);
                    rotation_estimates[i].y += ABS(rotated_gradient.x) + ABS(rotated_gradient.y);
                } else if ((rotated_gradient.x ^ rotated_gradient.y) < 0) {
                    rotation_estimates[i].x += ABS(rotated_gradient.y);
                    rotation_estimates[i].y += ABS(rotated_gradient.x);
                } else {
                    rotation_estimates[i].x += ABS(rotated_gradient.x);
                    rotation_estimates[i].y += ABS(rotated_gradient.y);
                }
            }
        }
    }
    float similarity[N_ROTATION_VECTORS];
    for(int16_t i = 0; i < N_ROTATION_VECTORS; ++i) {
        assert(rotation_estimates[i].x >= 0);
        assert(rotation_estimates[i].y >= 0);
        similarity[i] = cosine(rotation_estimates[i], (Vector2){1, 1});
        assert(similarity[i] >= 0);
        rotation_estimates[i] = rotate_vector(rotation_estimates[i], (Vector2){ROTATION_VECTORS[i].x, -ROTATION_VECTORS[i].y});
        if (((rotation_estimates[i].x ^ rotation_estimates[i].y) < 0 && rotation_estimates[i].y != 0)
                || rotation_estimates[i].x == 0) {
            int32_t x = rotation_estimates[i].x;
            rotation_estimates[i].x = ABS(rotation_estimates[i].y);
            rotation_estimates[i].y = ABS(x);
        } else {
            rotation_estimates[i].x = ABS(rotation_estimates[i].x);
            rotation_estimates[i].y = ABS(rotation_estimates[i].y);
        }
    }
    Vector2 rotation_average = {
        rotation_estimates[0].x + rotation_estimates[2].x,
        rotation_estimates[0].y + rotation_estimates[2].y,
    };
    int16_t best_estimate = 0;
    int16_t second_best_estimate = 0;
    for(int16_t i = 0; i < N_ROTATION_VECTORS; ++i) {
        if(similarity[i] >= similarity[best_estimate]) {
            second_best_estimate = best_estimate;
            best_estimate = i;
        } else if (similarity[i] >= second_best_estimate) {
            second_best_estimate = i;
        }
        printf("i %d s %f be %d sb %d bs %f c %d\n", i, similarity[i], best_estimate,second_best_estimate, similarity[best_estimate], squared_distance(rotation_estimates[i], rotation_average));
    }
    if(ABS(similarity[best_estimate] - similarity[second_best_estimate]) < 0.05) {
        int32_t da = squared_distance(rotation_estimates[second_best_estimate], rotation_average);
        int32_t db = squared_distance(rotation_estimates[best_estimate],  rotation_average);
        if(da < db) {
            best_estimate = second_best_estimate;
        }
    }

    //best_estimate = 3;
    //return rotation_average;
    return rotation_estimates[best_estimate]; 
}

float test_func(const ImageMatrix mat) {
    Vector2 rotation = estimate_rotation(mat);
    return rotation.x;
}

float test_func_2(const ImageMatrix mat) {
    Vector2 rotation = estimate_rotation(mat);
    return rotation.y;
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

void hough_line_transform(ImageMatrix* dst, const ImageMatrix src) {
    FOR_EACH_ELEMENT(*dst) { ELEMENT(*dst, row, col) = 0; }
    float angle_resolution = M_PI * 0.5f / dst->n_rows;
    float scale_to_index = dst->n_cols / sqrtf((src.n_rows * src.n_rows) +
                                                 (src.n_cols * src.n_cols));
    for (uint16_t i = 0; i < dst->n_rows; ++i) {
        float sin = sinf(i * angle_resolution);
        float cos = cosf(i * angle_resolution);
        FOR_EACH_ELEMENT(src) {
            ELEMENT(*dst, i,
                    (int16_t)(((sin * row) + (cos * col)) * scale_to_index)) +=
                    ELEMENT(src, row, col);
        }
    }
    assert(count_negative_elements(*dst) == 0);
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

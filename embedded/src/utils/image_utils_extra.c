#include "image_utils_extra.h"
#include <assert.h>

void img_convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    dst->n_rows = src.n_rows - (kernel.n_rows & ~1);
    dst->n_cols = src.n_cols - (kernel.n_cols & ~1);
    FOR_EACH_PIXEL(*dst, ) {
        int32_t sum = 0;
        FOR_EACH_PIXEL(kernel, k_) {
            sum += PIXEL(kernel, k_row, k_col) * PIXEL(src, row + k_row, col + k_col);
        }
        PIXEL(*dst, row, col) = CLAMP_INT_RANGE(sum, PIXEL_TYPE);
    }
}

void img_hough_line_transform(ImageMatrix dst, const ImageMatrix src) {
    assert(sizeof(PIXEL_TYPE) > 1);
    img_fill(dst, 0);
    float angle_resolution = M_PI * 0.5f / dst.n_rows;
    float scale_to_index =
            dst.n_cols / sqrtf((src.n_rows * src.n_rows) + (src.n_cols * src.n_cols));
    for (uint16_t i = 0; i < dst.n_rows; ++i) {
        float sin = sinf(i * angle_resolution);
        float cos = cosf(i * angle_resolution);
        FOR_EACH_PIXEL(src, ) {
            PIXEL(dst, i, (int16_t)(((sin * row) + (cos * col)) * scale_to_index)) +=
                    PIXEL(src, row, col);
        }
    }
    assert(img_count_negative(dst) == 0);
}

void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src) {
    const rgb888_t* data_rgb888 = (rgb888_t*) src.data;
    int32_t data_len = src.n_rows * src.n_cols;
    for (int32_t i = 0; i < data_len; ++i) {
        dst->data[i] = (data_rgb888[i].r + data_rgb888[i].g + data_rgb888[i].b) / 3;
    }
    img_copy_dimensions(dst, src, 0);
}

void img_convert_uint8_to_int16(ImageMatrix mat) {
    uint8_t* data_uint8 = (uint8_t*) mat.data;
    for (int32_t i = mat.n_rows * mat.n_cols - 1; i >= 0; --i) {
        mat.data[i] = data_uint8[i];
    }
}

void img_convert_int16_to_uint8(ImageMatrix mat) {
    uint8_t* data_uint8 = (uint8_t*) mat.data;
    int32_t data_len = mat.n_rows * mat.n_cols;
    for (int32_t i = 0; i < data_len; ++i) {
        data_uint8[i] = mat.data[i];
    }
}

int32_t img_count_negative(ImageMatrix mat) {
    assert(IS_SIGNED(PIXEL_TYPE));
    int32_t count = 0;
    FOR_EACH_PIXEL(mat, ) { count += PIXEL(mat, row, col) < 0; }
    return count;
}

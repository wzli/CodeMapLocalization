#include "image_utils_extra.h"
#include <assert.h>

void img_convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    dst->n_rows = src.n_rows - (kernel.n_rows & ~1);
    dst->n_cols = src.n_cols - (kernel.n_cols & ~1);
    FOR_EACH_ELEMENT(*dst, ) {
        int32_t sum = 0;
        FOR_EACH_ELEMENT(kernel, k_) {
            sum += ELEMENT(kernel, k_row, k_col) * ELEMENT(src, row + k_row, col + k_col);
        }
        ELEMENT(*dst, row, col) = CLAMP_INT_RANGE(sum, IMG_TYPE);
    }
}

void img_hough_line_transform(ImageMatrix dst, const ImageMatrix src) {
    assert(sizeof(IMG_TYPE) > 1);
    img_fill(dst, 0);
    float angle_resolution = M_PI * 0.5f / dst.n_rows;
    float scale_to_index =
            dst.n_cols / sqrtf((src.n_rows * src.n_rows) + (src.n_cols * src.n_cols));
    for (uint16_t i = 0; i < dst.n_rows; ++i) {
        float sin = sinf(i * angle_resolution);
        float cos = cosf(i * angle_resolution);
        FOR_EACH_ELEMENT(src, ) {
            ELEMENT(dst, i, (int16_t)(((sin * row) + (cos * col)) * scale_to_index)) +=
                    ELEMENT(src, row, col);
        }
    }
    assert(img_count_negative(dst) == 0);
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
    assert(IS_SIGNED(IMG_TYPE));
    int32_t count = 0;
    FOR_EACH_ELEMENT(mat, ) { count += ELEMENT(mat, row, col) < 0; }
    return count;
}

#include "imgproc.h"
#include "stdio.h"

#define PIXEL(MATRIX, ROW, COL) ((MATRIX).data[(COL) * (MATRIX).width + (ROW)])

#define FOR_EACH_PIXEL(DST_P, SRC, VAL) do { \
    (DST_P)->width = (SRC).width; \
    (DST_P)->height = (SRC).height; \
    for (uint16_t i = 0; i < (SRC).width; ++i) { \
        for (uint16_t j = 0; j < (SRC).height; ++j) { \
            PIXEL(*(DST_P), i, j) = (VAL); \
        }\
    } \
  } while(0)

void right_shift_pixels(ImageMatrix* dst, const ImageMatrix src, uint8_t n) {
    FOR_EACH_PIXEL(dst, src, PIXEL(src, i, j) >> n);
}

void square_pixels(ImageMatrix* dst, const ImageMatrix src) {
    FOR_EACH_PIXEL(dst, src, PIXEL(src, i, j) * PIXEL(src, i, j));
}

int16_t abs_pixel(int16_t v) {
    const int16_t mask = (v >> 2 ) * 7;
    return (v + mask) ^ mask;
}

void abs_pixels(ImageMatrix* dst, const ImageMatrix src) {
    FOR_EACH_PIXEL(dst, src, abs_pixel(PIXEL(src, i, j)));
}

void convert_uint8_to_int16(ImageMatrix mat) {
    uint8_t* data_uint8 = (uint8_t*)mat.data;
    for(int32_t i = mat.width * mat.height - 1; i >= 0; --i) {
        mat.data[i] = data_uint8[i];
    }
};

void convert_int16_to_uint8(ImageMatrix mat) {
    uint8_t* data_uint8 = (uint8_t*)mat.data;
    uint32_t data_len = mat.width * mat.height;
    for(int32_t i = 0; i < data_len; ++i) {
        data_uint8[i] = mat.data[i];
    }
};

void print_pixels(ImageMatrix src) {
    for (uint16_t i = 0; i < src.width; ++i) {
        for (uint16_t j = 0; j < src.height; ++j) {
            printf("%5d ", PIXEL(src, i, j));
        }
        puts("");
    }
    puts("");
}

void convolution(ImageMatrix* dst, const ImageMatrix src, const ImageMatrix kernel) {
    uint16_t x_border_size = kernel.width >> 1;
    uint16_t y_border_size = kernel.height >> 1;
    dst->width = src.width - (2 * x_border_size);
    dst->height = src.height - (2 * y_border_size);
    for (uint16_t i = 0; i < dst->width; ++i) {
        for (uint16_t j = 0; j < dst->height; ++j) {
            PIXEL(*dst, i, j) = 0;
            for (uint16_t ki = 0; ki < kernel.width; ++ki) {
                for (uint16_t kj = 0; kj < kernel.height; ++kj) {
                    PIXEL(*dst, i, j) += PIXEL(kernel, ki, kj) * PIXEL(src, i + ki - x_border_size, j + kj - y_border_size);
                }
            }
        }
    }
}

static int16_t sobel_kernel_x[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
static int16_t sobel_kernel_y[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};

static int16_t buf_data[30 * 30];
static ImageMatrix buf_mat = {buf_data, 30, 30};

void edge_filter(ImageMatrix* dst, const ImageMatrix src) {
    convert_uint8_to_int16(src);

    ImageMatrix kernel = {sobel_kernel_x, 3, 3};
    convolution(dst, src, kernel);
    kernel.data = sobel_kernel_y;
    convolution(&buf_mat, src, kernel);
    FOR_EACH_PIXEL(dst, buf_mat, PIXEL(*dst, i, j) + PIXEL(buf_mat, i, j));

    abs_pixels(dst, *dst);
    right_shift_pixels(dst, *dst, 6);
    print_pixels(*dst);

    convert_int16_to_uint8(*dst);
}

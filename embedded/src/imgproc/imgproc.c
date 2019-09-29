#include "imgproc.h"
#include "stdio.h"

void print_pixels(Image src, uint8_t sign) {
    for (uint16_t i = 0; i < src.width; ++i) {
        for (uint16_t j = 0; j < src.height; ++j) {
            printf("%4d ", sign ? (int8_t) src.data[j * src.width + i] : src.data[j * src.width + i]);
        }
        puts("");
    }
    puts("");
}

void right_shift_pixels(Image* dst, const Image src, uint8_t n) {
    for (uint16_t i = 0; i < src.width; ++i) {
        for (uint16_t j = 0; j < src.height; ++j) {
            dst->data[j * src.height + i] = src.data[j * src.height + i] >> n;
        }
    }
    dst->width = src.width;
    dst->height = src.height;
}

void square_pixels(Image* dst, const Image src) {
    for (uint16_t i = 0; i < src.width; ++i) {
        for (uint16_t j = 0; j < src.height; ++j) {
            dst->data[j * src.height + i] =
                    ((int8_t) src.data[j * src.height + i] * (int8_t) src.data[j * src.height + i]) >> 7;
        }
    }
    dst->width = src.width;
    dst->height = src.height;
}

void convolution(Image* dst, const Image src, const Image kernel) {
    uint16_t x_border_size = kernel.width >> 1;
    uint16_t y_border_size = kernel.height >> 1;
    dst->width = src.width - (2 * x_border_size);
    dst->height = src.height - (2 * y_border_size);
    for (uint16_t i = 0; i < dst->width; ++i) {
        for (uint16_t j = 0; j < dst->height; ++j) {
            dst->data[j * dst->width + i] = 0;
            for (uint16_t ki = 0; ki < kernel.width; ++ki) {
                for (uint16_t kj = 0; kj < kernel.height; ++kj) {
                    dst->data[j * dst->width + i] +=
                            kernel.data[kj * kernel.width + ki] *
                            (src.data[(j + kj - y_border_size) * src.width + i + ki - x_border_size]);
                }
            }
        }
    }
}

static uint8_t sobel_kernel_x[3 * 3] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
static uint8_t sobel_kernel_y[3 * 3] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
static uint8_t buffers[30][2];

void edge_filter(Image* dst, const Image src) {
    Image buf0 = {buffers[0], 0, 0};
    Image buf1 = {buffers[1], 0, 0};
    Image kernel = {sobel_kernel_x, 3, 3};
    right_shift_pixels(&buf0, src, 7);
    convolution(dst, buf0, kernel);
    square_pixels(dst, *dst);
    kernel.data = sobel_kernel_y;
    convolution(&buf1, buf0, kernel);
    print_pixels(buf1, 1);
    square_pixels(&buf1, buf1);
    print_pixels(buf1, 0);
    // print_pixels(*dst, 0);
    for (uint16_t i = 0; i < buf1.width; ++i) {
        for (uint16_t j = 0; j < buf1.height; ++j) {
            dst->data[j * dst->width + i] += buf1.data[j * buf1.width + i];
            // dst->data[j * dst->width + i] = ~((dst->data[j * dst->width + i] > ) -1);
        }
    }
    // print_pixels(*dst, 0);
}

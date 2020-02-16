#pragma once
#include "generic_image_utils.h"

// basic type generic functions

// PIXEL(MAT, ROW, COL)
// IMG_COPY(DST, SRC)
// IMG_FILL(MAT, VAL)
// IMG_PIXEL_SUM(SUM, MAT)
// IMG_PIXEL_AVERAGE(AVG, MAT)
// IMG_PIXEL_MAX(MAX_VAL, MAT)
// IMG_PIXEL_MIN(MIN_VAL, MAT)
// IMG_THRESHOLD(DST, SRC, THRESH)
// IMG_NORMALIZE(DST, SRC)
// IMG_NORMALIZE_RANGE(DST, SRC, MIN, MAX)
// IMG_CROP(DST, SRC, TOP_LEFT)
// IMG_VFLIP(DST, SRC)
// IMG_HFLIP(DST, SRC)

// convolution kernels
static const ImageMatrixInt8 img_sharpen_kernel = {
        (int8_t[]){-1, -1, -1, -1, 9, -1, -1, -1, -1}, {3, 3}};
static const ImageMatrixInt8 img_edge_detect_kernel = {
        (int8_t[]){-1, -1, -1, -1, 8, -1, -1, -1, -1}, {3, 3}};
static const ImageMatrixInt8 img_sobel_x_kernel = {
        (int8_t[]){-1, 0, 1, -2, 0, 2, -1, 0, 1}, {3, 3}};
static const ImageMatrixInt8 img_sobel_y_kernel = {
        (int8_t[]){-1, -2, -1, 0, 0, 0, 1, 2, 1}, {3, 3}};
static const ImageMatrixInt8 img_laplacian_kernel = {
        (int8_t[]){0, 1, 0, 1, -4, 1, 0, 1, 0}, {3, 3}};

// filters (they also work inplace)
void img_convolution_filter(ImageMatrix* dst, const ImageMatrix src, const ImageMatrixInt8 kernel);
void img_max_filter(ImageMatrix* dst, const ImageMatrix src, uint16_t block_size);
void img_min_filter(ImageMatrix* dst, const ImageMatrix src, uint16_t block_size);
void img_median_filter(ImageMatrix* dst, const ImageMatrix src, ImageMatrix window);

// interpolation methods
typedef uint8_t (*ImageInterpolation)(const ImageMatrix mat, Vector2f position);
uint8_t img_nearest_interpolation(const ImageMatrix mat, Vector2f position);
uint8_t img_bilinear_interpolation(const ImageMatrix mat, Vector2f position);
uint8_t img_bicubic_interpolation(const ImageMatrix mat, Vector2f position);

// geometric transformations
void img_resize(ImageMatrix dst, const ImageMatrix src, ImageInterpolation interpolation);
void img_rotate(ImageMatrix dst, const ImageMatrix src, Vector2f rotation, uint8_t bg_fill,
        ImageInterpolation interpolation);
void img_affine_transform(ImageMatrix dst, const ImageMatrix src, Matrix2f transform,
        uint8_t bg_fill, ImageInterpolation interpolation);

// histogram thresholding
void img_histogram(uint32_t histogram[256], const ImageMatrix mat);
uint8_t img_compute_otsu_threshold(const uint32_t histogram[256]);

// drawing utilitites
void img_draw_line(ImageMatrix mat, ImagePoint from, ImagePoint to, uint8_t color, uint8_t width);
void img_draw_box(ImageMatrix mat, ImagePoint from, ImagePoint to, uint8_t color, uint8_t width);
void img_draw_polygon(
        ImageMatrix mat, const ImagePoint* vertices, uint8_t len, uint8_t color, uint8_t width);
void img_draw_regular_polygon(ImageMatrix mat, ImagePoint center, Vector2f center_to_vertex,
        uint8_t order, uint8_t color, uint8_t width);

// domain transforms
void img_fast_fourier_transform(ImageMatrixComplex mat);
void img_inverse_fast_fourier_transform(ImageMatrixComplex mat);
void img_hough_line_transform(ImageMatrixInt32 dst, const ImageMatrix src);
void img_l1_distance_transform(ImageMatrixInt32* dst, const ImageMatrix src);
// below transforms require one row of line buffer, Eg dst size = (size.x)(size.y + 1)
void img_square_distance_transform(ImageMatrixInt32* dst, const ImageMatrix src);

// format conversions
void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src);

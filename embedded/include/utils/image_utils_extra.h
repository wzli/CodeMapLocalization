#pragma once
#include "image_utils.h"

void img_hough_line_transform(ImageMatrix dst, const ImageMatrix src);

// convert to grayscale assuming data buffer is in following format
typedef struct {
    uint8_t r, g, b;
} rgb888_t;

void img_convert_from_rgb888(ImageMatrix* dst, const ImageMatrix src);

void img_convert_uint8_to_int16(ImageMatrix mat);
void img_convert_int16_to_uint8(ImageMatrix mat);

int32_t img_count_negative(ImageMatrix mat);

#pragma once
#include "image_utils.h"
#include "bitwise_utils.h"

typedef struct {
    uint32_t bits;
    uint32_t mask;
    uint16_t n_errors;
    uint16_t n_samples;
} AxisCode;

void img_min_filter_2x2(ImageMatrix* dst, const ImageMatrix src);

Vector2f img_estimate_rotation(const ImageMatrix mat);

float img_estimate_scale(const ImageMatrix mat);

void img_to_bm64(BitMatrix64 dst, BitMatrix64 mask, const ImageMatrix src, uint8_t low_thresh,
        uint8_t high_thresh);

void bm64_to_img(ImageMatrix* dst, const BitMatrix64 src, const BitMatrix64 mask);

void img_bit_matrix_conversion(BitMatrix32 dst, BitMatrix32 mask, const ImageMatrix src,
        uint8_t low_thresh, uint8_t high_thresh);

void bm32_extract_axis_codes(AxisCode* row_code, AxisCode* col_code, BitMatrix32 matrix,
        BitMatrix32 mask, uint8_t min_samples);

AxisCode bm32_extract_column_code(uint32_t initial_row_guess, const BitMatrix32 matrix,
        const BitMatrix32 mask, uint8_t min_row_samples);

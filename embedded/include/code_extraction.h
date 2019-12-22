#pragma once
#include "image_utils.h"
#include "bitwise_utils.h"

typedef struct {
    uint32_t bits;
    uint32_t mask;
    uint32_t n_errors;
    uint32_t n_samples;
} AxisCode;

void img_edge_hysteresis_threshold(ImageMatrix* dst, const ImageMatrix src, uint16_t edge_thresh,
        PIXEL_TYPE value_thresh, PIXEL_TYPE value_tolerance);

Vector2f img_estimate_rotation(const ImageMatrix mat);

float img_estimate_scale(const ImageMatrix mat);

void img_bit_matrix_conversion(BitMatrix32 dst, BitMatrix32 mask, const ImageMatrix src,
        PIXEL_TYPE low_thresh, PIXEL_TYPE high_thresh);

void bm32_extract_axis_codes(AxisCode* row_code, AxisCode* col_code, BitMatrix32 matrix,
        BitMatrix32 mask, uint8_t min_samples);

AxisCode bm32_extract_column_code(uint32_t initial_row_guess, const BitMatrix32 matrix,
        const BitMatrix32 mask, uint8_t min_row_samples);

#pragma once
#include "image_utils.h"
#include "bitwise_utils.h"

typedef struct {
    uint32_t bits;
    uint32_t mask;
} AxisCode;

Vector2f img_estimate_rotation(const ImageMatrix mat);

void img_bit_matrix_conversion(BitMatrix32 dst, BitMatrix32 mask, const ImageMatrix src,
        IMF_TYPE low_thresh, IMF_TYPE high_thresh);

void bm32_extract_axis_codes(
        AxisCode* row_code, AxisCode* col_code, BitMatrix32 matrix, BitMatrix32 mask);

AxisCode bm32_extract_column_code(
        uint32_t row_code_bits, const BitMatrix32 matrix, const BitMatrix32 mask);

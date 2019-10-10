#pragma once
#include "image_filter.h"
#include "bit_matrix/bit_matrix.h"

Vector2f cmf_estimate_rotation(const ImageMatrix mat);
void cmf_bit_matrix_conversion(BitMatrix32 dst, BitMatrix32 mask, const ImageMatrix src,
        IMF_TYPE low_thresh, IMF_TYPE high_thresh);

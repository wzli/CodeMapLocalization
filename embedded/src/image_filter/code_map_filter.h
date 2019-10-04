#pragma once
#include "image_filter.h"

Vector2f cmf_estimate_rotation(const ImageMatrix mat) {
    void cmf_rotated_bitmask(BitMatrix32 mask, const ImageMatrix src, Vector2f rotation);
    void cmf_bit_conversion(BitMatrix32 dst, BitMatrix32 mask, const ImageMatrix src,
            IMF_TYPE low_thresh, IMF_TYPE high_thresh);

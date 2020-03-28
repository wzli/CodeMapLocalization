#pragma once
#include "image_utils.h"
#include "bitwise_utils.h"

typedef union {
    uint64_t x64;
    uint32_t x32;
    uint32_t data[2];
} CodeBits;

typedef struct {
    CodeBits bits;
    CodeBits mask;
    uint16_t n_errors;
    uint16_t n_samples;
} AxisCode;

// Image to bit matrix
void img_hyper_sharpen(ImageMatrix* dst, const ImageMatrix src);

void img_to_bm32(BitMatrix32 dst, BitMatrix32 mask, const ImageMatrix src, uint8_t low_thresh,
        uint8_t high_thresh);
void img_to_bm64(BitMatrix64 dst, BitMatrix64 mask, const ImageMatrix src, uint8_t low_thresh,
        uint8_t high_thresh);

// bit matrix code extraction
void bm32_to_img(ImageMatrix* dst, const BitMatrix32 src, const BitMatrix32 mask);
void bm64_to_img(ImageMatrix* dst, const BitMatrix64 src, const BitMatrix64 mask);

void bm32_from_axiscodes(
        BitMatrix32 matrix, BitMatrix32 mask, const AxisCode* row_code, const AxisCode* col_code);
void bm64_from_axiscodes(
        BitMatrix64 matrix, BitMatrix64 mask, const AxisCode* row_code, const AxisCode* col_code);

void bm32_extract_axiscodes(AxisCode* row_code, AxisCode* col_code, BitMatrix32 matrix,
        BitMatrix32 mask, uint8_t min_samples);
void bm64_extract_axiscodes(AxisCode* row_code, AxisCode* col_code, BitMatrix64 matrix,
        BitMatrix64 mask, uint8_t min_samples);

AxisCode bm32_extract_column_code(uint32_t row_estimate, const BitMatrix32 matrix,
        const BitMatrix32 mask, uint8_t min_row_samples);
AxisCode bm64_extract_column_code(uint64_t row_estimate, const BitMatrix64 matrix,
        const BitMatrix64 mask, uint8_t min_row_samples);

// bit vector extraction
uint32_t estimate_bit_triplet_offset(
        uint8_t* offset, const uint32_t* bits, const uint32_t* mask, uint32_t len);

uint32_t downsample_triplet_code(uint32_t* dst_bits, uint32_t* dst_mask, uint32_t dst_len,
        const uint32_t* src_bits, const uint32_t* src_mask, uint32_t src_len, uint8_t offset);

uint8_t ac32_downsample(AxisCode* axiscode, float scale);
uint8_t ac64_downsample(AxisCode* axiscode, float scale);

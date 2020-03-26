#pragma once
#include "image_utils.h"
#include "bitwise_utils.h"

typedef struct {
    uint32_t bits;
    uint32_t mask;
    uint16_t n_errors;
    uint16_t n_samples;
} AxisCode32;

typedef struct {
    uint64_t bits;
    uint64_t mask;
    uint16_t n_errors;
    uint16_t n_samples;
} AxisCode64;

#define AXISCODE_COPY(DST, SRC)                                                         \
    ((DST).bits = (SRC).bits, (DST).mask = (SRC).mask, (DST).n_errors = (SRC).n_errors, \
            (DST).n_samples = (SRC).n_samples)

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
        BitMatrix32 dst, BitMatrix32 mask, const AxisCode32 row_code, const AxisCode32 col_code);
void bm64_from_axiscodes(
        BitMatrix64 dst, BitMatrix64 mask, const AxisCode64 row_code, const AxisCode64 col_code);

void bm32_extract_axiscodes(AxisCode32* row_code, AxisCode32* col_code, BitMatrix32 matrix,
        BitMatrix32 mask, uint8_t min_samples);
void bm64_extract_axiscodes(AxisCode64* row_code, AxisCode64* col_code, BitMatrix64 matrix,
        BitMatrix64 mask, uint8_t min_samples);

AxisCode32 bm32_extract_column_code(uint32_t row_estimate, const BitMatrix32 matrix,
        const BitMatrix32 mask, uint8_t min_row_samples);
AxisCode64 bm64_extract_column_code(uint64_t row_estimate, const BitMatrix64 matrix,
        const BitMatrix64 mask, uint8_t min_row_samples);

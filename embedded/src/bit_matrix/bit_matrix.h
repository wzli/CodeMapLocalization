#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef uint32_t BitMatrix32[32];

static inline bool bm32_get_bit(const BitMatrix32 matrix, uint8_t row, uint8_t col) {
    return (matrix[row] >> col) & 1u;
};

static inline void bm32_set_bit(BitMatrix32 matrix, uint8_t row, uint8_t col) {
    matrix[row] |= 1u << col;
};

static inline void bm32_clear_bit(BitMatrix32 matrix, uint8_t row, uint8_t col) {
    matrix[row] &= ~(1u << col);
};

uint8_t bit_sum(uint32_t x);

void bm32_extract_codes(
        uint32_t* row_code, uint32_t* col_code, BitMatrix32 matrix, BitMatrix32 mask);
void bm32_transpose32(BitMatrix32 matrix);

uint32_t bm32_extract_column_code(
        uint32_t* row_code, const BitMatrix32 matrix, const BitMatrix32 mask);

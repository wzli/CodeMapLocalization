#pragma once
#include <stdint.h>

typedef uint32_t BitMatrix32[32];

// bit vector operations

static inline void bv32_set_bit(uint32_t* vector, uint32_t k) {
    vector[k >> 5] |= 1 << (k & 0x1F);
}

static inline void bv32_clear_bit(uint32_t* vector, uint32_t k) {
    vector[k >> 5] &= ~(1 << (k & 0x1F));
}

static inline uint32_t bv32_get_bit(const uint32_t* vector, uint32_t k) {
    return (vector[k >> 5] >> (k & 0x1F)) & 1;
}

// bit matrix operations

static inline uint32_t bm32_get_bit(const BitMatrix32 matrix, uint8_t row, uint8_t col) {
    return (matrix[row] >> col) & 1u;
};

static inline void bm32_set_bit(BitMatrix32 matrix, uint8_t row, uint8_t col) {
    matrix[row] |= 1u << col;
};

static inline void bm32_clear_bit(BitMatrix32 matrix, uint8_t row, uint8_t col) {
    matrix[row] &= ~(1u << col);
};

void bm32_transpose32(BitMatrix32 matrix);

// bitwise operations

static inline uint32_t inverse_bits(uint32_t x, uint32_t n) {
    return ~(x | (~0 << n));
}

uint32_t reverse_bits(uint32_t x, uint32_t n);
uint8_t first_set_bit(uint32_t x);
uint8_t bit_sum(uint32_t x);

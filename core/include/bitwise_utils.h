#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef uint32_t BitMatrix32[32];
typedef uint64_t BitMatrix64[64];

// bit vector operations

static inline void bv32_set_bit(uint32_t* vector, uint8_t k) {
    vector[k >> 5] |= 1 << (k & 0x1F);
}

static inline void bv64_set_bit(uint64_t* vector, uint8_t k) {
    vector[k >> 6] |= 1ull << (k & 0x3F);
}

static inline void bv32_clear_bit(uint32_t* vector, uint8_t k) {
    vector[k >> 5] &= ~(1 << (k & 0x1F));
}

static inline void bv64_clear_bit(uint64_t* vector, uint8_t k) {
    vector[k >> 6] &= ~(1ull << (k & 0x3F));
}

static inline bool bv32_get_bit(const uint32_t* vector, uint8_t k) {
    return (vector[k >> 5] >> (k & 0x1F)) & 1;
}

static inline bool bv64_get_bit(const uint64_t* vector, uint8_t k) {
    return (vector[k >> 6] >> (k & 0x3F)) & 1;
}

// bit matrix operations

static inline bool bm32_get_bit(const BitMatrix32 matrix, uint8_t row, uint8_t col) {
    return (matrix[row] >> col) & 1u;
}

static inline bool bm64_get_bit(const BitMatrix64 matrix, uint8_t row, uint8_t col) {
    return (matrix[row] >> col) & 1u;
}

static inline void bm32_set_bit(BitMatrix32 matrix, uint8_t row, uint8_t col) {
    matrix[row] |= 1u << col;
}

static inline void bm64_set_bit(BitMatrix64 matrix, uint8_t row, uint8_t col) {
    matrix[row] |= 1ull << col;
}

static inline void bm32_clear_bit(BitMatrix32 matrix, uint8_t row, uint8_t col) {
    matrix[row] &= ~(1u << col);
}

static inline void bm64_clear_bit(BitMatrix64 matrix, uint8_t row, uint8_t col) {
    matrix[row] &= ~(1ull << col);
}

void bm32_transpose(BitMatrix32 matrix);
void bm64_transpose(BitMatrix64 matrix);

// bitwise operations

static inline uint32_t mask_bits(uint8_t n) {
    return ~0u >> (32 - n);
}

static inline uint32_t invert_bits(uint32_t x, uint8_t n) {
    return ~x & mask_bits(n);
}

uint32_t reverse_bits(uint32_t x, uint8_t n);

#define count_trailing_zeros(X) ((X) ? perfect_log2((X) & -(X)) : sizeof(X) * 8)
#define perfect_log2(X) (sizeof(X) <= sizeof(uint32_t) ? perfect_log2_32(X) : perfect_log2_64(X))
uint8_t perfect_log2_32(uint32_t x);
uint8_t perfect_log2_64(uint64_t x);

#define count_bits(X) (sizeof(X) <= sizeof(uint32_t) ? count_bits_32(X) : count_bits_64(X))
uint8_t count_bits_32(uint32_t x);
static inline uint8_t count_bits_64(uint64_t x) {
    return count_bits_32(x) + count_bits_32(x >> 32);
}

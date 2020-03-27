#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef uint32_t BitMatrix32[32];
typedef uint64_t BitMatrix64[64];

// bit vector operations

static inline void bv32_set_bit(uint32_t* vector, uint32_t k) {
    vector[k >> 5] |= 1 << (k & 0x1F);
}

static inline void bv32_clear_bit(uint32_t* vector, uint32_t k) {
    vector[k >> 5] &= ~(1 << (k & 0x1F));
}

static inline bool bv32_get_bit(const uint32_t* vector, uint32_t k) {
    return (vector[k >> 5] >> (k & 0x1F)) & 1;
}

static inline void bv32_clear_all(uint32_t* vector, uint32_t len) {
    for (uint32_t i = 0; i < (len >> 5); ++i) {
        vector[i] = 0;
    }
}

#define bv32_get_slice bv32_get_slice_32
uint32_t bv32_get_slice_32(const uint32_t* vector, uint32_t k, uint8_t n);
uint64_t bv32_get_slice_64(const uint32_t* vector, uint32_t k, uint8_t n);

uint32_t bv32_scale(
        uint32_t* dst, const uint32_t* src, uint32_t dst_len, uint32_t src_len, float scale);

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

#define mask_bits mask_bits_32
static inline uint32_t mask_bits_32(uint8_t n) {
    return ~0u >> (32 - n);
}
static inline uint64_t mask_bits_64(uint8_t n) {
    return ~0ull >> (64 - n);
}

#define invert_bits(X, N) \
    (sizeof(X) <= sizeof(uint32_t) ? invert_bits_32(X, N) : invert_bits_64(X, N))
static inline uint32_t invert_bits_32(uint32_t x, uint8_t n) {
    return ~x & mask_bits_32(n);
}
static inline uint64_t invert_bits_64(uint64_t x, uint8_t n) {
    return ~x & mask_bits_64(n);
}

#define reverse_bits(X, N) \
    (sizeof(X) <= sizeof(uint32_t) ? reverse_bits_32(X, N) : reverse_bits_64(X, N))
uint32_t reverse_bits_32(uint32_t x, uint8_t n);
uint64_t reverse_bits_64(uint64_t x, uint8_t n);

#define count_trailing_zeros(X) ((X) ? perfect_log2((X) & -(X)) : sizeof(X) * 8)
#define perfect_log2(X) (sizeof(X) <= sizeof(uint32_t) ? perfect_log2_32(X) : perfect_log2_64(X))
uint8_t perfect_log2_32(uint32_t x);
uint8_t perfect_log2_64(uint64_t x);

#define count_bits(X) (sizeof(X) <= sizeof(uint32_t) ? count_bits_32(X) : count_bits_64(X))
uint8_t count_bits_32(uint32_t x);
static inline uint8_t count_bits_64(uint64_t x) {
    return count_bits_32(x) + count_bits_32(x >> 32);
}

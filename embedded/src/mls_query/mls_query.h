#pragma once
#include <stdint.h>

#define MLSQ_NOT_FOUND (0xFFFF)

typedef struct {
    const uint32_t* sequence;
    const uint16_t* sorted_code_positions;
    const uint16_t sequence_length;
    const uint8_t code_length;
} MlsQueryIndex;

extern const uint64_t MLSQ_SEQUENCE_ID;
extern const MlsQueryIndex MLSQ_INDEX;

static inline void ba32_set_bit(uint32_t A[], uint32_t k) {
    A[k >> 5] |= 1 << (k & 0x1F);
}

static inline void ba32_clear_bit(uint32_t A[], uint32_t k) {
    A[k >> 5] &= ~(1 << (k & 0x1F));
}

static inline uint32_t ba32_get_bit(uint32_t A[], uint32_t k) {
    return (A[k >> 5] >> (k & 0x1F)) & 1;
}

static inline uint32_t inverse_bits(uint32_t v, uint32_t n) {
    return ~(v | (~0 << n));
}

static inline uint32_t reverse_bits(uint32_t v, uint32_t n) {
    v &= ~(~0 << n);
    unsigned int r = v & 1;
    for (v >>= 1; v; v >>= 1) {
        r <<= 1;
        r |= v & 1;
        n--;
    }
    return r << (n - 1);
}

uint32_t mlsq_position_to_code(const uint32_t* sequence, uint8_t code_length, uint32_t position);
uint16_t mlsq_code_to_position(const MlsQueryIndex query_index, uint32_t code);
uint16_t mlsq_sort_code_positions(MlsQueryIndex query_index);

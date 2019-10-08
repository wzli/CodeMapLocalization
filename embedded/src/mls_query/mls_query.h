#pragma once
#include <stdint.h>

#define MLSQ_NOT_FOUND (0xFFFF)

typedef struct {
    uint32_t* sequence;
    uint16_t* sorted_code_positions;
    uint16_t sequence_length;
    uint8_t code_length;
} MlsQueryIndex;

static inline uint8_t is_little_endian() {
    const uint16_t i = 1;
    return (*(uint8_t*)&i);
}

static inline void ba32_set_bit(uint32_t A[],  uint32_t k) {
     A[k >> 5] |= 1 << (k & 0x1F);
}

static inline void ba32_clear_bit(uint32_t A[],  uint32_t k) {
     A[k >> 5] &= ~(1 << (k & 0x1F));
}

static inline uint32_t ba32_get_bit(uint32_t A[],  uint32_t k) {
     return (A[k >> 5] >> (k & 0x1F)) & 1;
}

uint32_t mlsq_position_to_code(const uint32_t* sequence, uint8_t code_length, uint32_t position);
uint16_t mlsq_code_to_position(const MlsQueryIndex query_index, uint32_t code);
uint16_t mlsq_sort_code_positions(MlsQueryIndex query_index);

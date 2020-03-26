#pragma once
#include "bitwise_utils.h"

static const uint16_t MLSQ_NOT_FOUND = 0xFFFF;

typedef struct {
    const uint32_t* sequence;
    const uint16_t* sorted_code_positions;
    const uint16_t sequence_length;
    const uint8_t code_length;
} MlsIndex;

extern const uint64_t MLS_ID;
extern const MlsIndex MLS_INDEX;

uint16_t mlsq_position_from_code(const MlsIndex query_index, uint32_t code);
uint16_t mlsq_sort_code_positions(MlsIndex query_index);

static inline uint32_t mlsq_code_from_position(
        const uint32_t* sequence, uint8_t code_length, uint16_t position) {
    return bv32_get_slice(sequence, position, code_length);
}

static inline uint32_t mlsq_code_from_position_indexed(uint8_t code_length, uint16_t position) {
    return position + code_length >= MLS_INDEX.sequence_length
                   ? 0
                   : bv32_get_slice(MLS_INDEX.sequence, position, code_length);
}

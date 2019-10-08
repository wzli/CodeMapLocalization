#include "mls_query.h"
#include <stdlib.h>
#include <assert.h>

static MlsQueryIndex compare_query_index;
static int compare_positions(const void* position1, const void* position2) {
    assert(compare_query_index.sorted_code_positions);
    uint32_t code1 = mlsq_position_to_code(compare_query_index.sequence, compare_query_index.code_length, *(uint16_t*)position1);
    uint32_t code2 = mlsq_position_to_code(compare_query_index.sequence, compare_query_index.code_length, *(uint16_t*)position2);
    if(*(uint32_t*)position1 & (1 << 31)) {
        code1 = *(uint32_t*)position1 ^  (1 << 31);
    }
    return code1 == code2 ? 0 : code1 > code2 ? 1 : -1;
}

uint32_t mlsq_position_to_code(const uint32_t* sequence, uint8_t code_length, uint32_t position) {
    assert(is_little_endian());
    assert(code_length < 32);
    uint32_t index = position >> 5;
    uint8_t offset = position & 0x1F;
    uint32_t code = sequence[index] >> offset;
    if(offset + code_length > 32) {
        code |= sequence[index + 1] << (32 - offset);
    }
    return code & ~(~0 << code_length);
}

uint16_t mlsq_code_to_position(const MlsQueryIndex query_index, uint32_t code) {
    compare_query_index = query_index;
    code |= 1 << 31;
    const void* position = bsearch(&code, query_index.sorted_code_positions, query_index.sequence_length - query_index.code_length + 1, 2, compare_positions);
    return position ? *(uint16_t*)position : MLSQ_NOT_FOUND;
}

uint16_t mlsq_sort_code_positions(const MlsQueryIndex query_index) {
    assert(query_index.sequence_length >= query_index.code_length);
    uint16_t positions_length = query_index.sequence_length - query_index.code_length + 1;
    for(uint16_t i = 0; i <  positions_length - 1; ++i) {
       query_index.sorted_code_positions[i] = i;
    }
    compare_query_index = query_index;
    qsort(query_index.sorted_code_positions, positions_length, 2, compare_positions);
    return positions_length;
}


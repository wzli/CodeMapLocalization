#include "mls_query.h"
#include <assert.h>

uint32_t mlsq_code_from_position(const uint32_t* sequence, uint8_t code_length, uint16_t position) {
    assert(code_length > 0 && code_length <= 32);
    uint32_t index = position >> 5;
    uint8_t offset = position & 0x1F;
    uint32_t code = sequence[index] >> offset;
    if (offset + code_length > 32) {
        code |= sequence[index + 1] << (32 - offset);
    }
    return code & mask_bits(code_length);
}

uint16_t mlsq_position_from_code(const MlsIndex query_index, uint32_t code) {
    assert(query_index.sorted_code_positions);
    assert(query_index.sequence_length >= query_index.code_length);
    int32_t start = 0;
    int32_t end = query_index.sequence_length - query_index.code_length;
    while (start <= end) {
        uint16_t mid = (start + end) / 2;
        uint16_t mid_position = query_index.sorted_code_positions[mid];
        uint32_t mid_code = mlsq_code_from_position(
                query_index.sequence, query_index.code_length, mid_position);
        if (code > mid_code) {
            start = mid + 1;
        } else if (code < mid_code) {
            end = mid - 1;
        } else {
            return mid_position;
        }
    }
    return MLSQ_NOT_FOUND;
}

uint16_t mlsq_sort_code_positions(const MlsIndex query_index) {
    assert(query_index.sequence_length >= query_index.code_length);
    uint16_t* sorted_code_positions = (uint16_t*) query_index.sorted_code_positions;
    uint16_t positions_length = query_index.sequence_length - query_index.code_length + 1;
    for (int32_t key_position = 0; key_position < positions_length; ++key_position) {
        uint32_t key_code = mlsq_code_from_position(
                query_index.sequence, query_index.code_length, key_position);
        int32_t test_index = key_position - 1;
        for (; test_index >= 0; --test_index) {
            uint16_t test_position = sorted_code_positions[test_index];
            uint32_t test_code = mlsq_code_from_position(
                    query_index.sequence, query_index.code_length, test_position);
            if (test_code <= key_code) {
                break;
            }
            sorted_code_positions[test_index + 1] = test_position;
        }
        sorted_code_positions[test_index + 1] = key_position;
    }
    return positions_length;
}

#include "tests.h"
#include "mls_query.h"

int test_mls_query() {
    uint16_t n_pos = MLS_INDEX.sequence_length - MLS_INDEX.code_length + 1;

    for (uint16_t pos = 0; pos < n_pos; ++pos) {
        uint32_t code = mlsq_code_from_position(MLS_INDEX.sequence, MLS_INDEX.code_length, pos);
        uint16_t pos_ret = mlsq_position_from_code(MLS_INDEX, code);
        test_assert(pos == pos_ret);
    }

    for (uint16_t pos = 0; pos < n_pos; ++pos) {
        uint32_t code = mlsq_code_from_position(MLS_INDEX.sequence, MLS_INDEX.code_length, pos);
        uint32_t code32 = mlsq_code_from_position(MLS_INDEX.sequence, 32, pos);
        test_assert(code == (code32 & mask_bits(MLS_INDEX.code_length)));
    }
    return 0;
}

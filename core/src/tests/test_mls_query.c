#include "test_utils.h"
#include "bitwise_utils.h"
#include "mls_query.h"

static int test_mlsq_position_from_code() {
    uint16_t n_pos = MLS_INDEX.sequence_length - MLS_INDEX.code_length + 1;
    for (uint16_t pos = 0; pos < n_pos; ++pos) {
        uint32_t code = bv32_get_slice(MLS_INDEX.sequence, pos, MLS_INDEX.code_length);
        uint16_t pos_ret = mlsq_position_from_code(MLS_INDEX, code);
        test_assert(pos == pos_ret);
    }
    return 0;
}

static int test_mlsq_code_from_position() {
    uint16_t n_pos = MLS_INDEX.sequence_length - MLS_INDEX.code_length + 1;
    for (uint16_t pos = 0; pos < n_pos; ++pos) {
        uint32_t code = bv32_get_slice(MLS_INDEX.sequence, pos, MLS_INDEX.code_length);
        uint32_t code32 = bv32_get_slice(MLS_INDEX.sequence, pos, 32);
        test_assert(code == (code32 & mask_bits(MLS_INDEX.code_length)));
    }
    return 0;
}

int test_mls_query() {
    test_run(test_mlsq_position_from_code);
    test_run(test_mlsq_code_from_position);
    return 0;
}

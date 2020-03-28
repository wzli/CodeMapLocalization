#include "test_utils.h"
#include "location_decode.h"
#include "mls_query.h"

#define TEST_VECTOR_SIZE 20

static int test_ac32_next_valid_segment() {
    AxisCode test_code = {{0xFFFF00FF}, {0xFFFF00FF}, 0, 1};
    uint8_t valid_segment_length;
    valid_segment_length = ac32_next_valid_segment(&test_code, 1);
    test_assert(test_code.bits.x32 == 0xFFFF00FF);
    test_assert(test_code.mask.x32 == 0xFFFF00FF);
    test_assert(valid_segment_length == 8);
    valid_segment_length = ac32_next_valid_segment(&test_code, 9);
    test_assert(valid_segment_length == 16);
    test_assert(test_code.bits.x32 == 0x0000FFFF);
    test_assert(test_code.mask.x32 == 0x0000FFFF);
    test_code = (AxisCode){{~0}, {~0}, 0, 1};
    test_assert(ac32_next_valid_segment(&test_code, 15) == 32);
    test_assert(test_code.bits.x32 == ~0u);
    test_assert(test_code.mask.x32 == ~0u);
    test_code = (AxisCode){{0}, {0}, 0, 1};
    test_assert(ac32_next_valid_segment(&test_code, 15) == 0);
    test_assert(test_code.bits.x32 == 0);
    test_assert(test_code.mask.x32 == 0);
    return 0;
}

static int test_ac64_next_valid_segment() {
    AxisCode test_code = {{.x64 = 0xFFFF00FFull << 32}, {.x64 = 0xFFFF00FFull << 32}, 0, 1};
    uint8_t valid_segment_length;
    valid_segment_length = ac64_next_valid_segment(&test_code, 1);
    test_assert(test_code.bits.x64 == 0xFFFF00FF);
    test_assert(test_code.mask.x64 == 0xFFFF00FF);
    test_assert(valid_segment_length == 8);
    valid_segment_length = ac64_next_valid_segment(&test_code, 9);
    test_assert(valid_segment_length == 16);
    test_assert(test_code.bits.x64 == 0x0000FFFF);
    test_assert(test_code.mask.x64 == 0x0000FFFF);
    test_code = (AxisCode){{~0ull}, {~0ull}, 0, 1};
    test_assert(ac64_next_valid_segment(&test_code, 15) == 64);
    test_assert(test_code.bits.x64 == ~0ull);
    test_assert(test_code.mask.x64 == ~0ull);
    test_code = (AxisCode){{0}, {0}, 0, 1};
    test_assert(ac64_next_valid_segment(&test_code, 15) == 0);
    test_assert(test_code.bits.x64 == 0);
    test_assert(test_code.mask.x64 == 0);
    return 0;
}

static int test_location_decode_blocks() {
    BitMatrix32 matrix, matrix_mask;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1000; src_col_pos < 1000 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = bv32_get_slice(MLS_INDEX.sequence, src_row_pos, 32);
            uint32_t src_col_code = bv32_get_slice(MLS_INDEX.sequence, src_col_pos, 32);
            AxisCode row_code = {{.x32 = src_row_code}, {~0ull}, 0, 0};
            AxisCode col_code = {{.x32 = src_col_code}, {~0ull}, 0, 0};
            bm32_from_axiscodes(matrix, matrix_mask, &row_code, &col_code);
            bm32_extract_axiscodes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = ac32_decode_position(row_code);
            AxisPosition col_pos = ac32_decode_position(col_code);
            test_assert(row_code.bits.x32 == src_row_code ||
                        invert_bits(row_code.bits.x32, 32) == src_row_code);
            test_assert(col_code.bits.x32 == src_col_code ||
                        invert_bits(col_code.bits.x32, 32) == src_col_code);
            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(src_row_pos + row_pos.span / 2 == row_pos.center);
            test_assert(src_col_pos + col_pos.span / 2 == col_pos.center);
        }
    return 0;
}

static int test_location_decode_reversed_blocks() {
    BitMatrix32 matrix, matrix_mask;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1000; src_col_pos < 1000 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = bv32_get_slice(MLS_INDEX.sequence, src_row_pos, 32);
            uint32_t src_col_code = bv32_get_slice(MLS_INDEX.sequence, src_col_pos, 32);
            src_row_code = reverse_bits(src_row_code, 32);
            src_col_code = reverse_bits(src_col_code, 32);
            AxisCode row_code = {{.x32 = src_row_code}, {~0ull}, 0, 0};
            AxisCode col_code = {{.x32 = src_col_code}, {~0ull}, 0, 0};
            bm32_from_axiscodes(matrix, matrix_mask, &row_code, &col_code);
            bm32_extract_axiscodes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = ac32_decode_position(row_code);
            AxisPosition col_pos = ac32_decode_position(col_code);
            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.reversed == 1);
            test_assert(col_pos.reversed == 1);
            test_assert(row_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(src_row_pos + (row_pos.span + 1) / 2 == row_pos.center);
            test_assert(src_col_pos + (col_pos.span + 1) / 2 == col_pos.center);
        }
    return 0;
}

static int test_location_decode_punctured_blocks() {
    BitMatrix32 matrix, matrix_mask;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1000; src_col_pos < 1000 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = bv32_get_slice(MLS_INDEX.sequence, src_row_pos, 32);
            uint32_t src_col_code = bv32_get_slice(MLS_INDEX.sequence, src_col_pos, 32);
            for (uint8_t i = 0; i < 32; ++i) {
                matrix_mask[i] = ~0xA;
                matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
                matrix[i] &= matrix_mask[i];
            }
            AxisCode row_code, col_code;
            bm32_extract_axiscodes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = ac32_decode_position(row_code);
            AxisPosition col_pos = ac32_decode_position(col_code);
            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.reversed == 0);
            test_assert(col_pos.reversed == 0);
            test_assert(row_pos.span + MLS_INDEX.code_length == 33 - 4);
            test_assert(col_pos.span + MLS_INDEX.code_length == 33);
            test_assert(src_row_pos + row_pos.span / 2 + 4 == row_pos.center);
            test_assert(src_col_pos + col_pos.span / 2 == col_pos.center);
        }
    return 0;
}

static int test_location_decode_truncated_blocks() {
    BitMatrix32 matrix, matrix_mask;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1000; src_col_pos < 1000 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = bv32_get_slice(MLS_INDEX.sequence, src_row_pos, 32);
            uint32_t src_col_code = bv32_get_slice(MLS_INDEX.sequence, src_col_pos, 32);
            for (uint8_t i = 0; i < 32; ++i) {
                matrix_mask[i] = ~0x400000F;
                matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
                matrix[i] &= matrix_mask[i];
            }
            AxisCode row_code, col_code;
            bm32_extract_axiscodes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = ac32_decode_position(row_code);
            AxisPosition col_pos = ac32_decode_position(col_code);
            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.reversed == 0);
            test_assert(col_pos.reversed == 0);
            test_assert(row_pos.span == 33 - MLS_INDEX.code_length - 10);
            test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(src_row_pos + row_pos.span / 2 + 4 == row_pos.center);
            test_assert(src_col_pos + col_pos.span / 2 == col_pos.center);
        }
    return 0;
}

int test_location_decode() {
    test_run(test_ac32_next_valid_segment);
    test_run(test_ac64_next_valid_segment);
    test_run(test_location_decode_blocks);
    test_run(test_location_decode_reversed_blocks);
    test_run(test_location_decode_punctured_blocks);
    test_run(test_location_decode_truncated_blocks);
    return 0;
}

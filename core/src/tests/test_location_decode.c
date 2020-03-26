#include "test_utils.h"
#include "location_decode.h"
#include "mls_query.h"

#define TEST_VECTOR_SIZE 20

static int test_downsample_axiscode() {
    AxisCode64 axiscode = {0x71C71C71C71C71C7ull, ~0ull, 0, 0};
    axiscode = downsample_axiscode64(axiscode, 1.0f / 3);
    test_assert(axiscode.bits == 0x155555);
    test_assert(axiscode.mask == (1 << 21) - 1);
    return 0;
}

static int test_next_valid_code_segment() {
    AxisCode32 test_code = {0xFFFF00FF, 0xFFFF00FF, 0, 1};
    uint8_t valid_segment_length;
    valid_segment_length = next_valid_code_segment(&test_code, 1);
    test_assert(test_code.bits == 0xFFFF00FF);
    test_assert(test_code.mask == 0xFFFF00FF);
    test_assert(valid_segment_length == 8);
    valid_segment_length = next_valid_code_segment(&test_code, 9);
    test_assert(valid_segment_length == 16);
    test_assert(test_code.bits == 0x0000FFFF);
    test_assert(test_code.mask == 0x0000FFFF);
    test_code = (AxisCode32){~0, ~0, 0, 1};
    test_assert(next_valid_code_segment(&test_code, 15) == 32);
    test_assert(test_code.bits == ~0u);
    test_assert(test_code.mask == ~0u);
    test_code = (AxisCode32){0, 0, 0, 1};
    test_assert(next_valid_code_segment(&test_code, 15) == 0);
    test_assert(test_code.bits == 0);
    test_assert(test_code.mask == 0);
    return 0;
}

static int test_location_decode_blocks() {
    BitMatrix32 matrix, matrix_mask;
    for (uint32_t src_row_pos = 1000; src_row_pos < 1000 + TEST_VECTOR_SIZE; ++src_row_pos)
        for (uint32_t src_col_pos = 1000; src_col_pos < 1000 + TEST_VECTOR_SIZE; ++src_col_pos) {
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            AxisCode32 row_code = {src_row_code, ~0, 0, 0};
            AxisCode32 col_code = {src_col_code, ~0, 0, 0};
            bm32_from_axiscodes(matrix, matrix_mask, row_code, col_code);
            bm32_extract_axiscodes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = decode_axis_position(row_code);
            AxisPosition col_pos = decode_axis_position(col_code);
            test_assert(row_code.bits == src_row_code ||
                        invert_bits(row_code.bits, 32) == src_row_code);
            test_assert(col_code.bits == src_col_code ||
                        invert_bits(col_code.bits, 32) == src_col_code);
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
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            src_row_code = reverse_bits(src_row_code, 32);
            src_col_code = reverse_bits(src_col_code, 32);
            AxisCode32 row_code = {src_row_code, ~0, 0, 0};
            AxisCode32 col_code = {src_col_code, ~0, 0, 0};
            bm32_from_axiscodes(matrix, matrix_mask, row_code, col_code);
            bm32_extract_axiscodes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = decode_axis_position(row_code);
            AxisPosition col_pos = decode_axis_position(col_code);
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
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            for (uint8_t i = 0; i < 32; ++i) {
                matrix_mask[i] = ~0xA;
                matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
                matrix[i] &= matrix_mask[i];
            }
            AxisCode32 row_code, col_code;
            bm32_extract_axiscodes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = decode_axis_position(row_code);
            AxisPosition col_pos = decode_axis_position(col_code);
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
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            for (uint8_t i = 0; i < 32; ++i) {
                matrix_mask[i] = ~0x400000F;
                matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
                matrix[i] &= matrix_mask[i];
            }
            AxisCode32 row_code, col_code;
            bm32_extract_axiscodes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = decode_axis_position(row_code);
            AxisPosition col_pos = decode_axis_position(col_code);
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
    test_run(test_downsample_axiscode);
    test_run(test_next_valid_code_segment);
    test_run(test_location_decode_blocks);
    test_run(test_location_decode_reversed_blocks);
    test_run(test_location_decode_punctured_blocks);
    test_run(test_location_decode_truncated_blocks);
    return 0;
}

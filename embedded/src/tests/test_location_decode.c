#include "tests.h"
#include "location_decode.h"
#include "mls_query.h"

int test_next_valid_code_segment() {
    AxisCode test_code = {0xFFFF00FF, 0xFFFF00FF};
    uint8_t valid_segment_length;
    valid_segment_length = next_valid_code_segment(&test_code, 1);
    test_assert(test_code.bits == 0xFFFF00FF);
    test_assert(test_code.mask == 0xFFFF00FF);
    test_assert(valid_segment_length == 8);
    valid_segment_length = next_valid_code_segment(&test_code, 9);
    test_assert(valid_segment_length == 16);
    test_assert(test_code.bits == 0x0000FFFF);
    test_assert(test_code.mask == 0x0000FFFF);
    test_code = (AxisCode){~0, ~0};
    test_assert(next_valid_code_segment(&test_code, 15) == 32);
    test_assert(test_code.bits == ~0);
    test_assert(test_code.mask == ~0);
    test_code = (AxisCode){0, 0};
    test_assert(next_valid_code_segment(&test_code, 15) == 0);
    test_assert(test_code.bits == 0);
    test_assert(test_code.mask == 0);
    return 0;
}

int test_location_decode() {
    static const uint32_t src_row_pos = 1000;
    static const uint32_t src_col_pos = 2000;
    BitMatrix32 matrix, matrix_mask;

    // first test, full 32 by 32 view
    uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
    uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
    for (uint8_t i = 0; i < 32; ++i) {
        matrix_mask[i] = ~0;
        matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
    }

    AxisCode row_code, col_code;
    bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask);
    AxisPosition row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
    AxisPosition col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);

    test_assert(row_code.bits == src_row_code || invert_bits(row_code.bits, 32) == src_row_code);
    test_assert(col_code.bits == src_col_code || invert_bits(col_code.bits, 32) == src_col_code);
    test_assert(row_pos.inverted == col_pos.inverted);
    test_assert(row_pos.span == 33 - MLS_INDEX.code_length);
    test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
    test_assert(src_row_pos == row_pos.start);
    test_assert(src_col_pos == col_pos.start);

    // second test, reversed axis
    src_row_code = reverse_bits(src_row_code, 32);
    src_col_code = reverse_bits(src_col_code, 32);

    for (uint8_t i = 0; i < 32; ++i) {
        matrix_mask[i] = ~0;
        matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
    }

    bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask);
    row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
    col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);

    test_assert(row_pos.inverted == col_pos.inverted);
    test_assert(row_pos.reversed == 1);
    test_assert(col_pos.reversed == 1);
    test_assert(row_pos.span == 33 - MLS_INDEX.code_length);
    test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
    test_assert(src_row_pos == row_pos.start + 1 - row_pos.span);
    test_assert(src_col_pos == col_pos.start + 1 - col_pos.span);

    // third test, normal axis with invalid segments
    src_row_code = reverse_bits(src_row_code, 32);
    src_col_code = reverse_bits(src_col_code, 32);

    for (uint8_t i = 0; i < 32; ++i) {
        matrix_mask[i] = ~0xA;
        matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
        matrix[i] &= matrix_mask[i];
    }

    bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask);
    row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
    col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);

    test_assert(row_pos.inverted == col_pos.inverted);
    test_assert(row_pos.reversed == 0);
    test_assert(col_pos.reversed == 0);
    test_assert(row_pos.span == 33 - MLS_INDEX.code_length - 4);
    test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
    test_assert(src_row_pos == row_pos.start - 4);
    test_assert(src_col_pos == col_pos.start);

    // 4th test, normal axis with invalid segments and boundaries
    for (uint8_t i = 0; i < 32; ++i) {
        matrix_mask[i] = ~0x400000F;
        matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
        matrix[i] &= matrix_mask[i];
    }

    bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask);
    row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
    col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);

    test_assert(row_pos.inverted == col_pos.inverted);
    test_assert(row_pos.reversed == 0);
    test_assert(col_pos.reversed == 0);
    test_assert(row_pos.span == 33 - MLS_INDEX.code_length - 10);
    test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
    test_assert(src_row_pos == row_pos.start - 4);
    test_assert(src_col_pos == col_pos.start);

    return 0;
}

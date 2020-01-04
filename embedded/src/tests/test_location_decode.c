#include "tests.h"
#include "location_decode.h"
#include "mls_query.h"

int test_next_valid_code_segment() {
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

int test_downsample_axis_code() {
    AxisCode64 axis_code_64 = {0x71C71C71C71C71C7ull, ~0ull, 0, 0};
    AxisCode32 axis_code_32 = downsample_axis_code(axis_code_64);
    test_assert(axis_code_32.bits == 0x155555);
    test_assert(axis_code_32.mask == (1 << 21) - 1);
    return 0;
}

int test_code_extract_64() {
    BitMatrix64 matrix, matrix_mask;
    uint32_t src_row_pos = 1000;
    uint32_t src_col_pos = 1001;
    uint64_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
    src_row_code |= (uint64_t) mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos + 32)
                    << 32;
    uint64_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
    src_col_code |= (uint64_t) mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos + 32)
                    << 32;
    for (uint8_t i = 0; i < 64; ++i) {
        matrix_mask[i] = ~0ull;
        matrix[i] = src_row_code ^ -((uint64_t)(src_col_code >> i) & 1ull);
    }
    AxisCode64 col_code = bm64_extract_column_code(0, matrix, matrix_mask, 5);
    bm64_transpose(matrix);
    bm64_transpose(matrix_mask);
    AxisCode64 row_code = bm64_extract_column_code(col_code.bits, matrix, matrix_mask, 5);
    test_assert((row_code.bits == src_row_code && col_code.bits == src_col_code) ||
                (row_code.bits == ~src_row_code && col_code.bits == ~src_col_code));
    bm64_transpose(matrix);
    bm64_transpose(matrix_mask);
    bm64_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask, 5);
    test_assert((row_code.bits == src_row_code && col_code.bits == src_col_code) ||
                (row_code.bits == ~src_row_code && col_code.bits == ~src_col_code));
    bm64_from_axis_codes(matrix, matrix_mask, row_code, col_code);
    for (uint8_t i = 0; i < 64; ++i) {
        test_assert(matrix_mask[i] == ~0ull);
        test_assert(matrix[i] == src_row_code || matrix[i] == ~src_row_code);
    }
    return 0;
}

int test_location_decode() {
    BitMatrix32 matrix, matrix_mask;

    for (uint32_t src_row_pos = 1000; src_row_pos < 1051; ++src_row_pos)
        for (uint32_t src_col_pos = 1000; src_col_pos < 1051; ++src_col_pos) {
            // first test, full 32 by 32 view
            uint32_t src_row_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_row_pos);
            uint32_t src_col_code = mlsq_code_from_position(MLS_INDEX.sequence, 32, src_col_pos);
            for (uint8_t i = 0; i < 32; ++i) {
                matrix_mask[i] = ~0;
                matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
            }

            AxisCode32 row_code, col_code;
            bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask, 3);
            AxisPosition row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
            AxisPosition col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);

            test_assert(row_code.bits == src_row_code ||
                        invert_bits(row_code.bits, 32) == src_row_code);
            test_assert(col_code.bits == src_col_code ||
                        invert_bits(col_code.bits, 32) == src_col_code);
            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(src_row_pos + row_pos.span / 2 == row_pos.center);
            test_assert(src_col_pos + col_pos.span / 2 == col_pos.center);

            // second test, reversed axis
            src_row_code = reverse_bits(src_row_code, 32);
            src_col_code = reverse_bits(src_col_code, 32);

            for (uint8_t i = 0; i < 32; ++i) {
                matrix_mask[i] = ~0;
                matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
            }

            bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask, 3);
            row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
            col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);

// print_bit_matrix(matrix);
#if 0
            printf("r %d c %d\n", src_row_pos, src_col_pos);
            print_bits(src_col_code, 32);
            print_bits(col_code.bits, 32);
            print_axis_position(col_pos);
#endif

            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.reversed == 1);
            test_assert(col_pos.reversed == 1);
            test_assert(row_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(col_pos.span == 33 - MLS_INDEX.code_length);
            test_assert(src_row_pos + (row_pos.span + 1) / 2 == row_pos.center);
            test_assert(src_col_pos + (col_pos.span + 1) / 2 == col_pos.center);

            // third test, normal axis with invalid segments
            src_row_code = reverse_bits(src_row_code, 32);
            src_col_code = reverse_bits(src_col_code, 32);

            for (uint8_t i = 0; i < 32; ++i) {
                matrix_mask[i] = ~0xA;
                matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
                matrix[i] &= matrix_mask[i];
            }

            bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask, 3);
            row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
            col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);

            test_assert(row_pos.inverted == col_pos.inverted);
            test_assert(row_pos.reversed == 0);
            test_assert(col_pos.reversed == 0);
            test_assert(row_pos.span + MLS_INDEX.code_length == 33 - 4);
            test_assert(col_pos.span + MLS_INDEX.code_length == 33);
            test_assert(src_row_pos + row_pos.span / 2 + 4 == row_pos.center);
            test_assert(src_col_pos + col_pos.span / 2 == col_pos.center);

            // 4th test, normal axis with invalid segments and boundaries
            for (uint8_t i = 0; i < 32; ++i) {
                matrix_mask[i] = ~0x400000F;
                matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
                matrix[i] &= matrix_mask[i];
            }

            bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask, 3);
            row_pos = decode_axis_position(row_code, MLS_INDEX.code_length);
            col_pos = decode_axis_position(col_code, MLS_INDEX.code_length);

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

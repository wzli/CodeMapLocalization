#include "tests.h"
#include "decode_location.h"
#include "mls_query.h"

static BitMatrix32 matrix, matrix_mask;
static const uint32_t src_row_pos = 1000;
static const uint32_t src_col_pos = 2000;

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

int test_decode_location() {
    uint32_t src_row_code = mlsq_code_from_position(MLSQ_INDEX.sequence, 32, src_row_pos);
    uint32_t src_col_code = mlsq_code_from_position(MLSQ_INDEX.sequence, 32, src_col_pos);

    for (uint8_t i = 0; i < 32; ++i) {
        matrix_mask[i] = ~0;
        matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
        matrix[i] &= matrix_mask[i];
    }

#if 0
    print_bit_matrix(matrix_mask);
    puts("");
    print_bit_matrix(matrix);
    puts("");
#endif

    AxisCode row_code, col_code;
    bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask);

    test_assert(row_code.bits == src_row_code || inverse_bits(row_code.bits, 32) == src_row_code);
    test_assert(col_code.bits == src_col_code || inverse_bits(col_code.bits, 32) == src_col_code);

    uint16_t row_pos, col_pos;
    CodeVerdict row_verdict = decode_axis(&row_pos, row_code, MLSQ_INDEX.code_length);
    test_assert(src_row_pos == row_pos);
    CodeVerdict col_verdict = decode_axis(&col_pos, col_code, MLSQ_INDEX.code_length);

    printf("col code pos verdict %d src %d recovered %d\n", col_verdict, src_col_pos, col_pos);
    printf(" src ");
    print_bits(src_col_code, 32);
    printf("bits ");
    print_bits(col_code.bits, 32);
    printf("mask ");
    print_bits(col_code.mask, 32);

    printf("row code pos verdict %d src %d recovered %d\n", row_verdict, src_row_pos, row_pos);
    printf(" src ");
    print_bits(src_row_code, 32);
    printf("bits ");
    print_bits(row_code.bits, 32);
    printf("mask ");
    print_bits(row_code.mask, 32);

    test_assert(src_col_pos == col_pos);
    return 0;
}

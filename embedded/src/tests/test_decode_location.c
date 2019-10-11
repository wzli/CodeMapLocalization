#include "test_decode_location.h"
#include "decode_location.h"
#include "mls_query.h"

static BitMatrix32 matrix, matrix_mask;
static const uint32_t src_row_pos = 1000;
static const uint32_t src_col_pos = 2000;
static const uint8_t row_bits_offset = 5;
static const uint8_t col_bits_offset = 9;

char* test_decode_location() {
    uint32_t src_row_code =
            mlsq_code_from_position(MLSQ_INDEX.sequence, MLSQ_INDEX.code_length, src_row_pos);
    uint32_t src_col_code =
            mlsq_code_from_position(MLSQ_INDEX.sequence, MLSQ_INDEX.code_length, src_col_pos);

    for (int i = col_bits_offset; i < MLSQ_INDEX.code_length + col_bits_offset; ++i) {
        matrix_mask[i] =
                ~(~0 << (MLSQ_INDEX.code_length + row_bits_offset)) & (~0 << row_bits_offset);
        matrix[i] =
                (src_row_code << row_bits_offset) ^ -((src_col_code >> (i - col_bits_offset)) & 1);
        matrix[i] &= matrix_mask[i];
    }
    print_bit_matrix(matrix_mask);
    puts("");
    print_bit_matrix(matrix);
    puts("");

    AxisCode row_code, col_code;
    bm32_extract_axis_codes(&row_code, &col_code, matrix, matrix_mask);

    uint16_t row_pos, col_pos;
    CodeVerdict row_verdict = decode_axis(&row_pos, row_code, MLSQ_INDEX.code_length);
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

    mu_assert("Decoded row position mismatch", src_row_pos == row_pos);
    mu_assert("Decoded column position mismatch", src_col_pos == col_pos);
    return 0;
}

#include "mls_query/mls_query.h"
#include "bit_matrix/bit_matrix.h"
#include "debug_utils/debug_utils.h"
#include <stdio.h>

BitMatrix32 matrix, matrix_mask;
const uint32_t src_row_pos = 1000;
const uint32_t src_col_pos = 2000;
const uint8_t row_bits_offset = 5;
const uint8_t col_bits_offset = 9;

int main() {
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
    bm32_extract_codes(&row_code, &col_code, matrix, matrix_mask);

    uint16_t col_pos = mlsq_position_from_code(MLSQ_INDEX, col_code.bits);
    if (col_pos == MLSQ_NOT_FOUND) {
        col_code.bits = ~col_code.bits & ((1 << MLSQ_INDEX.code_length) - 1);
        row_code.bits = ~row_code.bits & ((1 << MLSQ_INDEX.code_length) - 1);
        col_pos = mlsq_position_from_code(MLSQ_INDEX, col_code.bits);
    }
    uint16_t row_pos = mlsq_position_from_code(MLSQ_INDEX, row_code.bits);

    printf("col code pos src %d recovered %d\n", src_col_pos, col_pos);
    printf(" src ");
    print_bits(src_col_code, 32);
    printf("bits ");
    print_bits(col_code.bits, 32);
    printf("mask ");
    print_bits(col_code.mask, 32);

    printf("row code pos src %d recovered %d\n", src_row_pos, row_pos);
    printf(" src ");
    print_bits(src_row_code, 32);
    printf("bits ");
    print_bits(row_code.bits, 32);
    printf("mask ");
    print_bits(row_code.mask, 32);
    return 0;
}

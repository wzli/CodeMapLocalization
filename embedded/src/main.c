#include "mls_query/mls_query.h"
#include "bit_matrix/bit_matrix.h"
#include "debug_utils/debug_utils.h"
#include <stdio.h>

BitMatrix32 matrix, matrix_mask;
const uint32_t src_row_pos = 1000;
const uint32_t src_col_pos = 2000;

int main() {
    uint32_t src_row_code =
            mlsq_code_from_position(MLSQ_INDEX.sequence, MLSQ_INDEX.code_length, src_row_pos);
    uint32_t src_col_code =
            mlsq_code_from_position(MLSQ_INDEX.sequence, MLSQ_INDEX.code_length, src_col_pos);
    for (int i = 0; i < MLSQ_INDEX.code_length; ++i) {
        matrix_mask[i] = (1 << MLSQ_INDEX.code_length) - 1;
        matrix[i] = src_row_code ^ -((src_col_code >> i) & 1);
        print_bits(matrix[i], MLSQ_INDEX.code_length);
    }
    uint32_t row_code, col_code;
    bm32_extract_codes(&row_code, &col_code, matrix, matrix_mask);

    uint16_t col_pos = mlsq_position_from_code(MLSQ_INDEX, col_code);
    if (col_pos == MLSQ_NOT_FOUND) {
        col_code = ~col_code & ((1 << MLSQ_INDEX.code_length) - 1);
        row_code = ~row_code & ((1 << MLSQ_INDEX.code_length) - 1);
        col_pos = mlsq_position_from_code(MLSQ_INDEX, col_code);
    }
    uint16_t row_pos = mlsq_position_from_code(MLSQ_INDEX, row_code);

    printf("col code pos src %d recovered %d\n", src_col_pos, col_pos);
    print_bits(src_col_code, MLSQ_INDEX.code_length);
    print_bits(col_code, MLSQ_INDEX.code_length);

    printf("row code pos src %d recovered %d\n", src_row_pos, row_pos);
    print_bits(src_row_code, MLSQ_INDEX.code_length);
    print_bits(row_code, MLSQ_INDEX.code_length);
    return 0;
}

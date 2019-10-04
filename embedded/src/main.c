#include "lut/lut.h"
#include "bit_matrix/bit_matrix.h"

#include <stdio.h>

void print_bits(uint32_t word, int8_t word_length) {
    for (word_length--; word_length >= 0; word_length--) {
        printf("%u", (word >> word_length) & 1);
    }
    puts("");
}

BitMatrix32 matrix, matrix_mask;
const uint32_t row_entry = 1000;
const uint32_t col_entry = 2000;

int main() {
    for (int i = 0; i < LUT_KEY_LENGTH; ++i) {
        matrix_mask[i] = (1 << LUT_KEY_LENGTH) - 1;
        matrix[i] =
                LUT_DATA[row_entry].key ^ -((LUT_DATA[col_entry].key >> i) & 1);
        print_bits(matrix[i], LUT_KEY_LENGTH);
    }
    uint32_t row_code, col_code;
    extract_codes(&row_code, &col_code, matrix, matrix_mask);

    uint16_t col_pos = lut_search(LUT_DATA, LUT_SIZE, col_code);
    if (col_pos == LUT_KEY_ERROR) {
        col_code = ~col_code & ((1 << LUT_KEY_LENGTH) - 1);
        row_code = ~row_code & ((1 << LUT_KEY_LENGTH) - 1);
        col_pos = lut_search(LUT_DATA, LUT_SIZE, col_code);
    }
    uint16_t row_pos = lut_search(LUT_DATA, LUT_SIZE, row_code);

    printf("col code pos %d %d\n", LUT_DATA[col_entry].value, col_pos);
    print_bits(LUT_DATA[col_entry].key, LUT_KEY_LENGTH);
    print_bits(col_code, LUT_KEY_LENGTH);

    printf("row code pos %d %d\n", LUT_DATA[row_entry].value, row_pos);
    print_bits(LUT_DATA[row_entry].key, LUT_KEY_LENGTH);
    print_bits(row_code, LUT_KEY_LENGTH);
    return 0;
}

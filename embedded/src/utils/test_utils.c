#include "test_utils.h"

int test_count = 0;
char test_error[200] = "ALL TESTS PASSED";

void print_bits(uint32_t word, int8_t word_length) {
    for (word_length--; word_length >= 0; word_length--) {
        printf(" %u", (word >> word_length) & 1);
    }
    puts("");
}

void print_bit_matrix(BitMatrix32 matrix) {
    for (uint8_t row = 0; row < 32; ++row) {
        print_bits(matrix[row], 32);
    }
}

void print_image_matrix(ImageMatrix src) {
    for (int16_t row = 0; row < src.n_rows; ++row) {
        for (int16_t col = 0; col < src.n_cols; ++col) {
            printf("%6d ", ELEMENT(src, row, col));
        }
        puts("");
    }
    puts("");
}

void print_axis_position(AxisPosition position) {
    printf("start %d span %d inverted %d reversed %d\n", position.start, position.span,
            position.inverted, position.reversed);
}

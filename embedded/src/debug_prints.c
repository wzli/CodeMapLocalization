#include "debug_prints.h"
#include "stdio.h"

void print_bits(uint64_t word, int8_t word_length) {
    for (word_length--; word_length >= 0; word_length--) {
        printf(" %llu", (word >> word_length) & 1);
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
            printf("%6d ", PIXEL(src, row, col));
        }
        puts("");
    }
    puts("");
}

void print_axis_code(AxisCode32 axis_code) {
    uint8_t leading_zeros = count_trailing_zeros(reverse_bits(axis_code.mask, 32));
    printf("samples %d errors %d\n", axis_code.n_samples, axis_code.n_errors);
    print_bits(axis_code.bits, 32 - leading_zeros);
}

void print_axis_position(AxisPosition position) {
    printf("center %d span %d inverted %d reversed %d\n", position.center, position.span,
            position.inverted, position.reversed);
}

void print_location(Location location) {
    printf("x %d y %d c %.3f s %.3f match %d\n", location.x, location.y,
            (double) location.rotation.x, (double) location.rotation.y, location.match_size);
}

// below are used for python access

Vector2f test_add_angle(Vector2f rot_a, Vector2f rot_b) {
    return v2f_add_angle(rot_a, rot_b);
}

uint8_t test_diff_bits(uint32_t a, uint32_t b) {
    uint8_t diff = count_bits(a ^ b);
    return diff > 16 ? 32 - diff : diff;
}

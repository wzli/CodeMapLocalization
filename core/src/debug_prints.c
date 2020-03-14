#include "debug_prints.h"
#include <stdio.h>

void print_bits(uint64_t word, int8_t word_length) {
    for (word_length--; word_length >= 0; word_length--) {
        printf(" %u", (uint32_t)(word >> word_length) & 1);
    }
    puts("");
}

void print_bit_matrix(BitMatrix32 matrix) {
    for (uint8_t row = 0; row < 32; ++row) {
        print_bits(matrix[row], 32);
    }
}

void print_image_matrix(ImageMatrix src) {
    for (int16_t row = 0; row < src.size.y; ++row) {
        for (int16_t col = 0; col < src.size.x; ++col) {
            printf("%6d ", PIXEL(src, row, col));
        }
        puts("");
    }
    puts("");
}

void print_axiscode(AxisCode32 axiscode) {
    uint8_t leading_zeros = count_trailing_zeros(reverse_bits(axiscode.mask, 32));
    printf("samples %d errors %d\n", axiscode.n_samples, axiscode.n_errors);
    print_bits(axiscode.bits, 32 - leading_zeros);
}

void print_axis_position(AxisPosition position) {
    printf("center %d span %d inverted %d reversed %d\n", position.center, position.span,
            position.inverted, position.reversed);
}

void print_location(const Location* location) {
    printf("x %d y %d c %.3f s %.3f match %d\n", location->x, location->y,
            (double) location->rotation.x, (double) location->rotation.y, location->match_size);
}

void bm64_save_to_pgm(BitMatrix64 bit_matrix, BitMatrix64 bit_mask, const char* file_name) {
    ImageMatrix image = {(uint8_t[64 * 64]){}, {{64, 64}}};
    bm64_to_img(&image, bit_matrix, bit_mask);
    img_save_to_pgm(image, file_name);
}

// below are used for python access

Vector2f test_add_angle(Vector2f rot_a, Vector2f rot_b) {
    return (Vector2f)(rot_a.z * rot_b.z);
}

uint8_t test_diff_bits(uint32_t a, uint32_t b) {
    uint8_t diff = count_bits(a ^ b);
    return diff > 16 ? 32 - diff : diff;
}

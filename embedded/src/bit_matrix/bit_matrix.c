#include "bit_matrix/bit_matrix.h"

uint8_t bit_sum(uint32_t x) {
    x = x - ((x >> 1) & 0x55555555);
    x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
    return (((x + (x >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

void transpose32(BitMatrix32 A) {
    uint32_t m = 0xFFFF0000;
    for (uint32_t j = 16; j != 0; j = j >> 1, m = m ^ (m >> j)) {
        for (uint32_t k = 0; k < 32; k = (k + j + 1) & ~j) {
            uint32_t t = (A[k] ^ (A[k + j] << j)) & m;
            A[k] ^= t;
            A[k + j] ^= (t >> j);
        }
    }
}

uint32_t extract_column_code(uint32_t* row_code, const BitMatrix32 matrix, const BitMatrix32 mask) {
    uint32_t column_code = 0;
    for (uint8_t i = 0; i < 32; ++i) {
        uint8_t row_diff = bit_sum((*row_code ^ matrix[i]) & mask[i]);
        uint32_t inv_bits = -(2 * row_diff > bit_sum(mask[i]));
        column_code |= (inv_bits & 1) << i;
        *row_code &= ~mask[i];
        *row_code |= (matrix[i] ^ inv_bits) & mask[i];
    }
    return column_code;
}

void extract_codes(uint32_t* row_code, uint32_t* col_code, BitMatrix32 matrix, BitMatrix32 mask) {
    *col_code = extract_column_code(row_code, matrix, mask);
    transpose32(matrix);
    transpose32(mask);
    *row_code = extract_column_code(col_code, matrix, mask);
}

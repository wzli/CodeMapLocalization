#include "bit_matrix.h"

// bunch of bit twiddling hacks
// see https://graphics.stanford.edu/~seander/bithacks.html

void bm32_transpose32(BitMatrix32 A) {
    uint32_t m = 0xFFFF0000;
    for (uint32_t j = 16; j != 0; j = j >> 1, m = m ^ (m >> j)) {
        for (uint32_t k = 0; k < 32; k = (k + j + 1) & ~j) {
            uint32_t t = (A[k] ^ (A[k + j] << j)) & m;
            A[k] ^= t;
            A[k + j] ^= (t >> j);
        }
    }
}

uint8_t first_set_bit(uint32_t x) {
    static const int MultiplyDeBruijnBitPosition[32] = {0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15,
            25, 17, 4, 8, 31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9};
    return MultiplyDeBruijnBitPosition[((uint32_t)((x & -x) * 0x077CB531U)) >> 27];
}

uint8_t bit_sum(uint32_t x) {
    x = x - ((x >> 1) & 0x55555555);
    x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
    return (((x + (x >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

AxisCode bm32_extract_column_code(
        uint32_t row_code_bits, const BitMatrix32 matrix, const BitMatrix32 mask) {
    AxisCode column_code = {};
    for (uint8_t i = 0; i < 32; ++i) {
        uint8_t mask_sum = bit_sum(mask[i]);
        if (mask_sum == 0) {
            continue;
        }
        uint8_t row_diff = bit_sum((row_code_bits ^ matrix[i]) & mask[i]);
        uint32_t inv_bits = -(2 * row_diff > mask_sum);
        column_code.bits |= (inv_bits & 1) << i;
        column_code.mask |= 1 << i;
        row_code_bits &= ~mask[i];
        row_code_bits |= (inv_bits ^ matrix[i]) & mask[i];
    }
    return column_code;
}

void bm32_extract_codes(
        AxisCode* row_code, AxisCode* col_code, BitMatrix32 matrix, BitMatrix32 mask) {
    *col_code = bm32_extract_column_code(row_code->bits, matrix, mask);
    bm32_transpose32(matrix);
    bm32_transpose32(mask);
    *row_code = bm32_extract_column_code(col_code->bits, matrix, mask);
    uint8_t offset = first_set_bit(col_code->mask);
    col_code->bits >>= offset;
    col_code->mask >>= offset;
    offset = first_set_bit(row_code->mask);
    row_code->bits >>= offset;
    row_code->mask >>= offset;
}

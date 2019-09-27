#include <stdio.h>
#include <stdlib.h>
#include "lut/lut.h"

void print_bits(uint32_t word, int8_t word_length) {
    for(word_length--; word_length >= 0; word_length--) {
        printf("%u", (word >> word_length) & 1);
    }
    puts("");
}

uint8_t bit_sum(uint32_t i) {
    i = i - ((i >> 1) & 0x55555555);
    i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
    return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

void transpose32(uint32_t* A) {
    uint32_t m = 0xFFFF0000;
    for(uint32_t j = 16; j != 0; j = j >> 1, m = m ^ (m >> j)) {
        for(uint32_t k = 0; k < 32; k = (k + j + 1) & ~j) {
            uint32_t t = (A[k] ^ (A[k+j] << j)) & m;
            A[k] ^= t;
            A[k + j] ^= (t >> j);
        }
    }
}

uint32_t extract_column_code(uint32_t* code_block, uint32_t* block_mask, uint32_t length) {
    uint32_t column_code = 0;
    for(int i = 1; i < length; ++i) {
        uint32_t mask = block_mask[i] & block_mask[i-1];
        uint32_t row_diff = (code_block[i] ^ code_block[i-1]) & mask;
        column_code |= (column_code << 1) & (1 << i);
        column_code ^= (bit_sum(row_diff) > (bit_sum(mask) >> 1)) << i;
    }
    return column_code;
}

int main() {
    uint32_t* code_block = malloc(sizeof(uint32_t) * 32);
    uint32_t* block_mask = malloc(sizeof(uint32_t) * 32);

    print_bits(LUT_DATA[0].key, LUT_KEY_LENGTH);
    print_bits(LUT_DATA[1].key, LUT_KEY_LENGTH);
    printf("hello, world, %d\n", bit_sum(LUT_DATA[1].key));

    for(int i = 0; i < LUT_KEY_LENGTH; ++i) {
        block_mask[i] = (1 << LUT_KEY_LENGTH) - 1;
        code_block[i] = LUT_DATA[0].key ^ -((LUT_DATA[1].key >> i) & 1);
        print_bits(code_block[i], LUT_KEY_LENGTH);
    }

    uint32_t column_code = extract_column_code(code_block, block_mask, LUT_KEY_LENGTH);
    printf("col code ");
    print_bits(column_code, LUT_KEY_LENGTH);

    transpose32(code_block);
    transpose32(block_mask);

    uint32_t row_code = extract_column_code(code_block, block_mask, LUT_KEY_LENGTH);
    printf("row code ");
    print_bits(row_code, LUT_KEY_LENGTH);

    free(block_mask);
    free(code_block);
    return 0;
}

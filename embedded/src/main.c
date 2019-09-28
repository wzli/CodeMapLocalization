#include "lut/lut.h"
#include <stdio.h>

void print_bits(uint32_t word, int8_t word_length) {
  for (word_length--; word_length >= 0; word_length--) {
    printf("%u", (word >> word_length) & 1);
  }
  puts("");
}

uint8_t bit_sum(uint32_t x) {
  x = x - ((x >> 1) & 0x55555555);
  x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
  return (((x + (x >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

void transpose32(uint32_t A[32]) {
  uint32_t m = 0xFFFF0000;
  for (uint32_t j = 16; j != 0; j = j >> 1, m = m ^ (m >> j)) {
    for (uint32_t k = 0; k < 32; k = (k + j + 1) & ~j) {
      uint32_t t = (A[k] ^ (A[k + j] << j)) & m;
      A[k] ^= t;
      A[k + j] ^= (t >> j);
    }
  }
}

uint32_t extract_column_code(uint32_t *code_block, uint32_t *code_block_mask,
                             uint32_t n_rows, uint32_t row_code) {
  uint32_t column_code = 0;
  for (uint8_t i = 0; i < n_rows; ++i) {
    uint8_t row_diff = bit_sum((row_code ^ code_block[i]) & code_block_mask[i]);
    uint32_t inv_bits = -(2 * row_diff > bit_sum(code_block_mask[i]));
    column_code |= (inv_bits & 1) << i;
    row_code &= ~code_block_mask[i];
    row_code |= (code_block[i] ^ inv_bits) & code_block_mask[i];
  }
  return column_code;
}

uint32_t code_block[32], code_block_mask[32];

const uint32_t row_entry = 1000;
const uint32_t column_entry = 2000;

int main() {
  for (int i = 0; i < LUT_KEY_LENGTH; ++i) {
    code_block_mask[i] = (1 << LUT_KEY_LENGTH) - 1;
    code_block[i] =
        LUT_DATA[row_entry].key ^ -((LUT_DATA[column_entry].key >> i) & 1);
    print_bits(code_block[i], LUT_KEY_LENGTH);
  }

  uint32_t column_code =
      extract_column_code(code_block, code_block_mask, LUT_KEY_LENGTH, 0);
  transpose32(code_block);
  transpose32(code_block_mask);
  uint32_t row_code = extract_column_code(code_block, code_block_mask,
                                          LUT_KEY_LENGTH, column_code);

  printf("col code pos %d %d\n", LUT_DATA[column_entry].value,
         lut_search(LUT_DATA, LUT_SIZE, column_code));
  print_bits(LUT_DATA[column_entry].key, LUT_KEY_LENGTH);
  print_bits(column_code, LUT_KEY_LENGTH);

  printf("row code pos %d %d\n", LUT_DATA[row_entry].value,
         lut_search(LUT_DATA, LUT_SIZE, row_code));
  print_bits(LUT_DATA[row_entry].key, LUT_KEY_LENGTH);
  print_bits(row_code, LUT_KEY_LENGTH);
  return 0;
}

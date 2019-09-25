#include "seq.h"
#include "lut.h"
#include <stdio.h>

uint16_t inv_lut[1 << 13] = {};
uint16_t lut[1 << 13] = {};
uint16_t test_lut[1 << 16] = {};

uint64_t hash_salts[1 << 13];

void print_word(uint16_t word) {
  for(uint8_t i = 0; i < WORD_LENGTH; ++i) {
      printf("%u", (word >> i) & 1);
  }
  printf("\r\n");
}

int main(int argc, char **argv) {
  printf("%ld %ld\r\n", sizeof(sequence), sizeof(lut));
  for(uint16_t i = 0, word = 0; sequence[i]; ++i) {
      word >>= 1;
      if(sequence[i] == '1') {
          word |= 1 << (WORD_LENGTH - 1);
      }
      int position = i - WORD_LENGTH + 1;
      if(position > 0) {
          inv_lut[position] = word;
          test_lut[word] = position;
          //printf("added %u ", position);
          //print_word(word);
      }
  };

  uint16_t longest_chain = 0;
  uint64_t salt = 1l << (64 - 13);
  for(uint16_t i = 1; inv_lut[i]; ++i) {
      uint16_t hash = lut_hash(13, salt, inv_lut[i]);
      if(salt == hash_salts[hash]) {
          if(i > longest_chain) {
              longest_chain = i;
              printf("salt %lu at %u\r\n", salt, i);
          }
          //printf("hash %u salt %lu at %u\r\n", hash, salt, i);
          i = 1;
          ++salt;
      }
      hash_salts[hash] = salt;
  }

  for(uint16_t i = 1; inv_lut[i]; ++i) {
      if(LUT_SUCCESS != lut_insert(lut, 13, salt, inv_lut[i], i)) {
          printf("collision in lut at %u\r\n", i);
          break;
      }
  }
  printf("salt %lu \r\n", salt);
  return 0;
}

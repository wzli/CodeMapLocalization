#include "seq.h"
#include "lut.h"
#include <stdio.h>

#define LUT_SIZE (sizeof(sequence) - WORD_LENGTH)

LutEntry lut[LUT_SIZE];
uint16_t test_lut[1 << 16] = {};

void print_word(uint16_t word) {
  for(uint8_t i = 0; i < WORD_LENGTH; ++i) {
      printf("%u", (word >> i) & 1);
  }
  printf("\r\n");
}

int main(int argc, char **argv) {
  printf("sequence len %ld lut size %ld\r\n", sizeof(sequence), sizeof(lut));
  for(uint16_t i = 0, word = 0; sequence[i]; ++i) {
      word >>= 1;
      if(sequence[i] == '1') {
          word |= 1 << (WORD_LENGTH - 1);
      }
      int position = i - WORD_LENGTH + 1;
      if(position >= 0) {
          test_lut[word] = position;
          lut[position] = (LutEntry){word, position};
          //printf("added %u ", position);
          //print_word(word);
      }
  };
  lut_sort(lut, LUT_SIZE);
  for(uint32_t i = 0; i < (1 << 16); ++i) {
      uint16_t position = lut_search(lut, LUT_SIZE, i);
      if(position != LUT_KEY_ERROR && position != test_lut[i]) {
          printf("mismatch key %u actual %u found %u\r\n", i, test_lut[i], position);
          return -1;
      }
  }

  FILE *fptr;
  fptr = fopen("lut_dat.h", "w");

  if(fptr == NULL)
  {
      printf("Error opening file\r\n");
      return -2;
  }

  fprintf(fptr, "#pragma once\r\n");
  fprintf(fptr, "#include \"lut.h\"\r\n\r\n");
  fprintf(fptr, "const LutEntry lut[] = {\r\n");
  for(uint16_t i = 0; i < LUT_SIZE; ++i) {
      fprintf(fptr, "  {%#x, %u},\r\n", lut[i].key, lut[i].value);
  }
  fprintf(fptr, "\r\n};");

  fclose(fptr);

  return 0;
}

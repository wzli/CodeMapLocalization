#pragma once
#include "image_filter.h"
#include "bit_matrix.h"

#define mu_assert(message, test) do { if (!(test)) return message; } while (0)
#define mu_run_test(test) do { char *message = test(); tests_run++; \
    if (message) return message; } while (0)
extern int tests_run;

void print_bits(uint32_t word, int8_t word_length);
void print_bit_matrix(BitMatrix32 matrix);
void print_image_matrix(ImageMatrix src);

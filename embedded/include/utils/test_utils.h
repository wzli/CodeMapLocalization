#pragma once
#include "image_utils.h"
#include "bitwise_utils.h"
#include "location_decode.h"
#include <stdio.h>

// unit test defines
#define test_assert(COND)                                                                         \
    do {                                                                                          \
        int error = !(COND);                                                                      \
        if (error) {                                                                              \
            sprintf(test_error, "FAILED %s\n  %s:%d\n  %s", __func__, __FILE__, __LINE__, #COND); \
            return error;                                                                         \
        }                                                                                         \
    } while (0)
#define test_run(test)        \
    do {                      \
        test_count++;         \
        int error = (test)(); \
        if (error)            \
            return error;     \
    } while (0)
extern int test_count;
extern char test_error[];

// debug prints for internal data structures
void print_bits(uint32_t word, int8_t word_length);
void print_bit_matrix(BitMatrix32 matrix);
void print_image_matrix(ImageMatrix src);
void print_axis_position(AxisPosition position);

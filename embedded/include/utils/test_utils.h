#pragma once
#include "image_utils.h"
#include "bitwise_utils.h"
#include "location_decode.h"
#include <stdio.h>

// unit test defines
#define test_success(COND)                                                           \
    do {                                                                             \
        int error = (COND);                                                          \
        if (error) {                                                                 \
            printf("FAIL %s\n  %s:%d\n  %s\n", __func__, __FILE__, __LINE__, #COND); \
            return error;                                                            \
        }                                                                            \
    } while (0)

#define test_assert(COND) test_success(!(COND))

#define test_run(TEST)                     \
    do {                                   \
        printf("RUNNING %s ...\n", #TEST); \
        int error = (TEST)();              \
        if (error) {                       \
            return error;                  \
        }                                  \
        printf("PASS\n");                  \
    } while (0)

// debug prints for internal data structures
void print_bits(uint32_t word, int8_t word_length);
void print_bit_matrix(BitMatrix32 matrix);
void print_image_matrix(ImageMatrix src);
void print_axis_code(AxisCode32 axis_code);
void print_axis_position(AxisPosition position);
void print_location(Location location);

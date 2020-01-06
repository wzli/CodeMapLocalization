#pragma once
#include "image_utils.h"
#include "bitwise_utils.h"
#include "location_decode.h"

// debug prints for internal data structures
void print_bits(uint64_t word, int8_t word_length);
void print_bit_matrix(BitMatrix32 matrix);
void print_image_matrix(ImageMatrix src);
void print_axis_code(AxisCode32 axis_code);
void print_axis_position(AxisPosition position);
void print_location(Location location);
void img_save_to_pgm(ImageMatrix image, const char* file_name);
void bm64_save_to_pgm(BitMatrix64 bit_matrix, BitMatrix64 bit_mask, const char* file_name);

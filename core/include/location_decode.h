#pragma once
#include "code_extraction.h"

typedef struct {
    uint16_t center;
    uint8_t span : 6;
    uint8_t inverted : 1;
    uint8_t reversed : 1;
} AxisPosition;

typedef struct {
    uint16_t x;
    uint16_t y;
    Vector2f rotation;
    int16_t match_size;
} Location;

uint8_t next_valid_code_segment(AxisCode32* axiscode, uint8_t code_length);
AxisPosition decode_axis_position(AxisCode32 axiscode, uint8_t code_length);
Location deduce_location(AxisPosition row_position, AxisPosition col_position);
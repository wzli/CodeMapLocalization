#pragma once
#include "code_extraction.h"

typedef struct {
    uint16_t start;
    uint8_t span : 6;
    uint8_t inverted : 1;
    uint8_t reversed : 1;
} AxisPosition;

uint8_t next_valid_code_segment(AxisCode* axis_code, uint8_t code_length);
AxisPosition decode_axis_position(AxisCode axis_code, uint8_t code_length);

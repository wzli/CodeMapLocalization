#pragma once
#include "code_extraction.h"

typedef enum {
    CODE_VERDICT_DIRECT,
    CODE_VERDICT_INVERSE,
    CODE_VERDICT_REVERSE,
    CODE_VERDICT_INVERSE_REVERSE,
    CODE_VERDICT_ERROR,
} CodeVerdict;

uint8_t next_valid_code_segment(AxisCode* axis_code, uint8_t code_length);
CodeVerdict decode_axis(uint16_t* output_position, AxisCode axis_code, uint8_t code_length);

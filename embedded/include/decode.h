#pragma once
#include "bit_matrix.h"

typedef enum {
    CODE_VERDICT_DIRECT,
    CODE_VERDICT_INVERSE,
    CODE_VERDICT_REVERSE,
    CODE_VERDICT_INVERSE_REVERSE,
    CODE_VERDICT_ERROR,
} CodeVerdict;

CodeVerdict decode_axis(uint16_t* output_position, AxisCode axis_code, uint8_t code_length);

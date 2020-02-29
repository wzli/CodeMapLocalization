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

typedef struct {
    AxisCode64 row_code;
    AxisCode64 col_code;
    float lower_bound;
    float upper_bound;
    float step_size;
} ScaleQuery;

typedef struct {
    Location location;
    AxisCode32 row_code;
    AxisCode32 col_code;
    float scale;
} ScaleMatch;

uint8_t next_valid_code_segment(AxisCode32* axiscode, uint8_t code_length);
AxisPosition decode_axis_position(AxisCode32 axiscode, uint8_t code_length);
Location deduce_location(AxisPosition row_position, AxisPosition col_position);

void scale_search_location(ScaleMatch* match, const ScaleQuery* query);

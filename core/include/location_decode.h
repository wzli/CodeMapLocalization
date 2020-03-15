#pragma once
#include "code_extraction.h"

typedef struct {
    uint16_t center;
    uint8_t span : 6;
    uint8_t inverted : 1;
    uint8_t reversed : 1;
} AxisPosition;

typedef struct {
    uint16_t match_size;
    uint16_t x;
    uint16_t y;
    uint8_t direction;
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

typedef struct {
    ScaleMatch filtered_match;
    uint16_t distance_threshold;
    uint16_t match_size_threshold;
    uint8_t bit_error_ratio_threshold;
    uint8_t max_rejection_count;
    uint8_t rejection_count;
} OutlierFilter;

uint8_t next_valid_code_segment(AxisCode32* axiscode, uint8_t code_length);
AxisPosition decode_axis_position(AxisCode32 axiscode);
Location deduce_location(AxisPosition row_position, AxisPosition col_position);

void scale_search_location(ScaleMatch* match, const ScaleQuery* query);

bool outlier_filter_location(OutlierFilter* filter, const ScaleMatch* new_match);

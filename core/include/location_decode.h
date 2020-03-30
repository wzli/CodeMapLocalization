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
    Location location;
    AxisCode row_code;
    AxisCode col_code;
    float scale;
    float quality;
    uint16_t bit_errors;
} ScaleMatch;

typedef struct {
    ScaleMatch filtered_match;
    float quality_threshold;
    uint16_t distance_threshold;
    uint16_t match_length_threshold;
    uint8_t xor_error_ratio_threshold;
    uint8_t max_rejection_count;
    uint8_t rejection_count;
} OutlierFilter;

uint8_t ac32_next_valid_segment(AxisCode* axiscode, uint8_t code_length);
uint8_t ac64_next_valid_segment(AxisCode* axiscode, uint8_t code_length);

AxisPosition ac32_decode_position(AxisCode axiscode, uint8_t stride);
AxisPosition ac64_decode_position(AxisCode axiscode, uint8_t stride);

void ac32_scale_search_location(
        ScaleMatch* match, const AxisCode* row_code, const AxisCode* col_code, float decay_rate);
void ac64_scale_search_location(
        ScaleMatch* match, const AxisCode* row_code, const AxisCode* col_code, float decay_rate);

Location deduce_location(AxisPosition row_position, AxisPosition col_position);

bool outlier_filter_location(OutlierFilter* filter, const ScaleMatch* new_match);

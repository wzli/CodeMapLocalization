#include "location_decode.h"
#include "mls_query.h"
#include <assert.h>

uint8_t next_valid_code_segment(AxisCode* axis_code, uint8_t code_length) {
    assert(axis_code);
    assert(code_length <= 32);
    uint8_t valid_segment_length = first_set_bit(~axis_code->mask);
    while (axis_code->mask && valid_segment_length < code_length) {
        axis_code->mask >>= valid_segment_length;
        axis_code->bits >>= valid_segment_length;
        valid_segment_length = first_set_bit(axis_code->mask);
        axis_code->mask >>= valid_segment_length;
        axis_code->bits >>= valid_segment_length;
        valid_segment_length = first_set_bit(~axis_code->mask);
    }
    return valid_segment_length;
};

AxisPosition decode_axis_position(AxisCode axis_code, uint8_t code_length) {
    const uint32_t code_mask = mask_bits(code_length);
    AxisPosition best_position = {};
    for (uint8_t valid_segment_length = next_valid_code_segment(&axis_code, code_length);
            valid_segment_length > 0;
            valid_segment_length = next_valid_code_segment(&axis_code, code_length)) {
        uint32_t code = axis_code.bits & code_mask;
        AxisPosition position = {};
        position.center = mlsq_position_from_code(MLS_INDEX, code);
        if (position.center == MLSQ_NOT_FOUND) {
            code = invert_bits(code, code_length);
            position.inverted = 1;
            position.center = mlsq_position_from_code(MLS_INDEX, code);
        }
        if (position.center == MLSQ_NOT_FOUND) {
            code = reverse_bits(code, code_length);
            position.reversed = 1;
            position.center = mlsq_position_from_code(MLS_INDEX, code);
        }
        if (position.center == MLSQ_NOT_FOUND) {
            code = invert_bits(code, code_length);
            position.inverted = 0;
            position.center = mlsq_position_from_code(MLS_INDEX, code);
        }
        position.span = position.center != MLSQ_NOT_FOUND;
        if (position.span && valid_segment_length > code_length) {
            uint32_t extended_code = position.inverted
                                             ? invert_bits(axis_code.bits, valid_segment_length)
                                             : axis_code.bits & mask_bits(valid_segment_length);
            uint32_t expected_code =
                    position.reversed
                            ? reverse_bits(
                                      mlsq_code_from_position(MLS_INDEX.sequence,
                                              valid_segment_length,
                                              position.center + code_length - valid_segment_length),
                                      valid_segment_length)
                            : mlsq_code_from_position(
                                      MLS_INDEX.sequence, valid_segment_length, position.center);
            uint8_t first_diff = first_set_bit(extended_code ^ expected_code);
            position.span += MIN(first_diff, valid_segment_length) - code_length;
        }
        if (position.span > best_position.span) {
            best_position = position;
        }
        if (position.span >= 32) {
            break;
        }
        position.span += !position.span;
        axis_code.bits >>= position.span;
        axis_code.mask >>= position.span;
    }
    best_position.center +=
            best_position.reversed ? 1 - best_position.span / 2 : best_position.span / 2;
    return best_position;
}

Location deduce_location(AxisPosition row_position, AxisPosition col_position) {
    Location location = {};
    if (row_position.inverted == col_position.inverted) {
        return location;
    }
    location.detection_size = row_position.span * col_position.span;
    assert(location.detection_size >= 0);
    if (location.detection_size == 0) {
        return location;
    }
    float z = 1 - 2 * row_position.reversed;
    if (row_position.reversed ^ col_position.reversed) {
        location.x = col_position.center;
        location.y = row_position.center;
        location.rotation.y = z;
    } else {
        location.x = row_position.center;
        location.y = col_position.center;
        location.rotation.x = z;
    }
    return location;
}

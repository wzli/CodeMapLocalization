#include "location_decode.h"
#include "mls_query.h"
#include <assert.h>

uint8_t next_valid_code_segment(AxisCode* axis_code, uint8_t code_length) {
    assert(axis_code);
    assert(code_length <= 32);
    uint8_t valid_segment_length = first_set_bit(~axis_code->mask);
    while (axis_code->mask > 0 && valid_segment_length < code_length) {
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
    AxisPosition best_position = {};
    const uint32_t code_mask = mask_bits(code_length);
    uint8_t valid_segment_length = next_valid_code_segment(&axis_code, code_length);
    while (valid_segment_length > 0) {
        uint32_t code = axis_code.bits & code_mask;
        AxisPosition position = {};
        position.start = mlsq_position_from_code(MLS_INDEX, code);
        if (position.start == MLSQ_NOT_FOUND) {
            code = invert_bits(code, code_length);
            position.inverted = 1;
            position.start = mlsq_position_from_code(MLS_INDEX, code);
        }
        if (position.start == MLSQ_NOT_FOUND) {
            code = reverse_bits(code, code_length);
            position.reversed = 1;
            position.start = mlsq_position_from_code(MLS_INDEX, code);
        }
        if (position.start == MLSQ_NOT_FOUND) {
            code = invert_bits(code, code_length);
            position.inverted = 0;
            position.start = mlsq_position_from_code(MLS_INDEX, code);
        }
        if (position.start == MLSQ_NOT_FOUND) {
            axis_code.bits >>= 1;
            axis_code.mask >>= 1;
            valid_segment_length = next_valid_code_segment(&axis_code, code_length);
            continue;
        }
        if (valid_segment_length == code_length) {
            position.span = 1;
            axis_code.bits >>= 1;
            axis_code.mask >>= 1;
        } else {
            uint32_t extended_code = position.inverted
                                             ? invert_bits(axis_code.bits, valid_segment_length)
                                             : axis_code.bits & mask_bits(valid_segment_length);
            uint32_t expected_code =
                    position.reversed
                            ? reverse_bits(
                                      mlsq_code_from_position(MLS_INDEX.sequence,
                                              valid_segment_length,
                                              position.start + code_length - valid_segment_length),
                                      valid_segment_length)
                            : mlsq_code_from_position(
                                      MLS_INDEX.sequence, valid_segment_length, position.start);
            uint32_t diff_code = extended_code ^ expected_code;
            uint8_t first_diff = first_set_bit(diff_code);
            position.span = valid_segment_length - code_length + 1 - sum_bits(diff_code);
            if (first_diff < 32) {
                axis_code.bits >>= first_diff;
                axis_code.mask >>= first_diff;
            } else {
                axis_code.bits = 0;
                axis_code.mask = 0;
            }
        }
        if (position.span > best_position.span) {
            best_position = position;
        }
        valid_segment_length = next_valid_code_segment(&axis_code, code_length);
    }
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
    switch ((row_position.reversed << 1) | col_position.reversed) {
        case 0:
            location.x = row_position.start + row_position.span / 2;
            location.y = col_position.start + col_position.span / 2;
            location.rotation = (Vector2f){1.0f, 0.0f};
            break;
        case 1:
            location.x = col_position.start + 1 - col_position.span / 2;
            location.y = row_position.start + row_position.span / 2;
            location.rotation = (Vector2f){0.0f, 1.0f};
            break;
        case 2:
            location.x = col_position.start + col_position.span / 2;
            location.y = row_position.start + 1 - row_position.span / 2;
            location.rotation = (Vector2f){0.0f, -1.0f};
            break;
        case 3:
            location.x = row_position.start + 1 - row_position.span / 2;
            location.y = col_position.start + 1 - col_position.span / 2;
            location.rotation = (Vector2f){-1.0f, 0.0f};
            break;
    }
    return location;
}

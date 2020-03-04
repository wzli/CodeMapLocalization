#include "location_decode.h"
#include "mls_query.h"
#include <assert.h>

uint8_t next_valid_code_segment(AxisCode32* axiscode, uint8_t code_length) {
    assert(axiscode);
    assert(code_length <= 32);
    uint8_t valid_segment_length = count_trailing_zeros(~axiscode->mask);
    while (axiscode->mask && valid_segment_length < code_length) {
        axiscode->mask >>= valid_segment_length;
        axiscode->bits >>= valid_segment_length;
        valid_segment_length = count_trailing_zeros(axiscode->mask);
        axiscode->mask >>= valid_segment_length;
        axiscode->bits >>= valid_segment_length;
        valid_segment_length = count_trailing_zeros(~axiscode->mask);
    }
    return valid_segment_length;
};

AxisPosition decode_axis_position(AxisCode32 axiscode) {
    AxisPosition best_position = {};
    const uint32_t code_mask = mask_bits(MLS_INDEX.code_length);
    for (uint8_t valid_segment_length = next_valid_code_segment(&axiscode, MLS_INDEX.code_length);
            valid_segment_length > 0;
            valid_segment_length = next_valid_code_segment(&axiscode, MLS_INDEX.code_length)) {
        uint32_t code = axiscode.bits & code_mask;
        AxisPosition position = {};
        position.center = mlsq_position_from_code(MLS_INDEX, code);
        if (position.center == MLSQ_NOT_FOUND) {
            code = invert_bits(code, MLS_INDEX.code_length);
            position.inverted = 1;
            position.center = mlsq_position_from_code(MLS_INDEX, code);
        }
        if (position.center == MLSQ_NOT_FOUND) {
            code = reverse_bits(code, MLS_INDEX.code_length);
            position.reversed = 1;
            position.center = mlsq_position_from_code(MLS_INDEX, code);
        }
        if (position.center == MLSQ_NOT_FOUND) {
            code = invert_bits(code, MLS_INDEX.code_length);
            position.inverted = 0;
            position.center = mlsq_position_from_code(MLS_INDEX, code);
        }
        position.span = position.center != MLSQ_NOT_FOUND;
        if (position.span && valid_segment_length > MLS_INDEX.code_length) {
            uint32_t extended_code = position.inverted
                                             ? invert_bits(axiscode.bits, valid_segment_length)
                                             : axiscode.bits & mask_bits(valid_segment_length);
            uint32_t expected_code =
                    position.reversed
                            ? reverse_bits(mlsq_code_from_position_indexed(valid_segment_length,
                                                   position.center + MLS_INDEX.code_length -
                                                           valid_segment_length),
                                      valid_segment_length)
                            : mlsq_code_from_position_indexed(
                                      valid_segment_length, position.center);
            uint8_t first_diff = count_trailing_zeros(extended_code ^ expected_code);
            position.span += MIN(first_diff, valid_segment_length) - MLS_INDEX.code_length;
        }
        if (position.span > best_position.span) {
            best_position = position;
        }
        if (position.span >= 32) {
            break;
        }
        position.span += !position.span;
        axiscode.bits >>= position.span;
        axiscode.mask >>= position.span;
    }
    best_position.center +=
            best_position.reversed ? 1 - best_position.span / 2 : best_position.span / 2;
    if (axiscode.n_errors * 4 > axiscode.n_samples) {
        best_position.span = 0;
    }
    return best_position;
}

void deduce_location(Location* location, AxisPosition row_position, AxisPosition col_position) {
    location->match_size = row_position.inverted == col_position.inverted
                                   ? row_position.span * col_position.span
                                   : 0;
    assert(location->match_size >= 0);
    float z = 1 - 2 * row_position.reversed;
    if (row_position.reversed ^ col_position.reversed) {
        location->x = col_position.center;
        location->y = row_position.center;
        location->rotation = (Vector2f){{0, z}};
    } else {
        location->x = row_position.center;
        location->y = col_position.center;
        location->rotation = (Vector2f){{z, 0}};
    }
}

void scale_search_location(ScaleMatch* match, const ScaleQuery* query) {
    assert(match && query);
    assert(query->lower_bound > 0 && query->upper_bound > 0 && query->step_size > 0);
    Location loc;
    for (float scale = query->lower_bound; scale <= query->upper_bound; scale += query->step_size) {
        // scale and down sample axis codes
        AxisCode64 scaled_row_code = scale_axiscode64(query->row_code, scale);
        AxisCode64 scaled_col_code = scale_axiscode64(query->col_code, scale);
        AxisCode32 row_code_32 = downsample_axiscode(scaled_row_code);
        AxisCode32 col_code_32 = downsample_axiscode(scaled_col_code);
        // decode posiiton
        AxisPosition row_pos = decode_axis_position(row_code_32);
        AxisPosition col_pos = decode_axis_position(col_code_32);
        deduce_location(&loc, row_pos, col_pos);
        if (loc.match_size >= match->location.match_size) {
            *match = (ScaleMatch){loc, row_code_32, col_code_32, scale};
        }
    }
}

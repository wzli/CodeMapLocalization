#include "location_decode.h"
#include "mls_query.h"
#include <assert.h>

#define WIDTH 32
#include "position_decode_impl.h"

#define WIDTH 64
#include "position_decode_impl.h"

AxisPosition decode_axis_position(AxisCode32 axiscode) {
    AxisPosition best_position = {0};
    const uint32_t code_mask = mask_bits(MLS_INDEX.code_length);
    for (uint8_t valid_segment_length = ac32_next_valid_segment(&axiscode, MLS_INDEX.code_length);
            valid_segment_length > 0;
            valid_segment_length = ac32_next_valid_segment(&axiscode, MLS_INDEX.code_length)) {
        uint32_t code = axiscode.bits & code_mask;
        AxisPosition position = {0};
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

Location deduce_location(AxisPosition row_position, AxisPosition col_position) {
    Location loc = {0};
    loc.match_size = row_position.inverted == col_position.inverted
                             ? row_position.span * col_position.span
                             : 0;
    assert(loc.match_size >= 0);
    loc.direction = (row_position.reversed << 1) | (col_position.reversed ^ row_position.reversed);
    if (row_position.reversed ^ col_position.reversed) {
        loc.x = col_position.center;
        loc.y = row_position.center;
    } else {
        loc.x = row_position.center;
        loc.y = col_position.center;
    }
    return loc;
}

void scale_search_location(ScaleMatch* match, const ScaleQuery* query) {
    assert(match && query);
    assert(query->lower_bound > 0 && query->upper_bound > 0 && query->step_size > 0);
    for (float scale = query->lower_bound; scale <= query->upper_bound; scale += query->step_size) {
        // scale and down sample axis codes
        AxisCode64 row_code = downsample_axiscode64(query->row_code, scale / 3);
        AxisCode64 col_code = downsample_axiscode64(query->col_code, scale / 3);
        AxisCode32 row_code_32, col_code_32;
        AXISCODE_COPY(row_code_32, row_code);
        AXISCODE_COPY(col_code_32, col_code);
        // decode posiiton
        AxisPosition row_pos = decode_axis_position(row_code_32);
        AxisPosition col_pos = decode_axis_position(col_code_32);
        Location loc = deduce_location(row_pos, col_pos);
        if (loc.match_size >= match->location.match_size) {
            *match = (ScaleMatch){loc, row_code_32, col_code_32, scale};
        }
    }
}

bool outlier_filter_location(OutlierFilter* ctx, const ScaleMatch* new_match) {
    assert(ctx && new_match);
    if (new_match->location.x == ctx->filtered_match.location.x &&
            new_match->location.y == ctx->filtered_match.location.y) {
        return true;
    }
    if (new_match->location.match_size < SQR(ctx->match_length_threshold)) {
        return false;
    }
    if ((new_match->row_code.n_errors * ctx->bit_error_ratio_threshold >
                new_match->row_code.n_samples) ||
            (new_match->col_code.n_errors * ctx->bit_error_ratio_threshold >
                    new_match->col_code.n_samples)) {
        return false;
    }
    int32_t dx = new_match->location.x - ctx->filtered_match.location.x;
    int32_t dy = new_match->location.y - ctx->filtered_match.location.y;
    if (ABS(dx) > ctx->distance_threshold || ABS(dy) > ctx->distance_threshold) {
        if (++ctx->rejection_count > ctx->max_rejection_count) {
            ctx->rejection_count = 0;
            ctx->filtered_match = *new_match;
            return true;
        }
        return false;
    }
    ctx->rejection_count = 0;
    ctx->filtered_match = *new_match;
    return true;
}

AxisCode64 downsample_axiscode64(AxisCode64 axiscode, float scale) {
    uint32_t bits[2] = {0};
    uint32_t mask[2] = {0};
    downsample_axiscode_64(bits, mask, scale, &axiscode);

    axiscode.bits = bits[1];
    axiscode.bits <<= 32;
    axiscode.bits |= bits[0];

    axiscode.mask = mask[1];
    axiscode.mask <<= 32;
    axiscode.mask |= mask[0];

    return axiscode;
}

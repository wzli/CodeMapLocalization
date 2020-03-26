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
}

AxisPosition decode_axis_position(AxisCode32 axiscode) {
    AxisPosition best_position = {0};
    const uint32_t code_mask = mask_bits(MLS_INDEX.code_length);
    for (uint8_t valid_segment_length = next_valid_code_segment(&axiscode, MLS_INDEX.code_length);
            valid_segment_length > 0;
            valid_segment_length = next_valid_code_segment(&axiscode, MLS_INDEX.code_length)) {
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
        AxisCode64 scaled_row_code = scale_axiscode64(query->row_code, scale);
        AxisCode64 scaled_col_code = scale_axiscode64(query->col_code, scale);
        AxisCode32 row_code_32 = downsample_axiscode(scaled_row_code);
        AxisCode32 col_code_32 = downsample_axiscode(scaled_col_code);
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
    assert(scale > 0 && scale <= 1);
    uint64_t scaled_bits[3] = {0};
    uint64_t scaled_mask[3] = {0};
    uint8_t dst_idx = 0;
    float src_inc = 1.0f / (3 * scale);
    for (float src_idx = 0; dst_idx < 64 * 3 && src_idx < 64; ++dst_idx, src_idx += src_inc) {
        if (bv64_get_bit(&axiscode.bits, (uint8_t) src_idx)) {
            bv64_set_bit(scaled_bits, dst_idx);
        }
        if (bv64_get_bit(&axiscode.mask, (uint8_t) src_idx)) {
            bv64_set_bit(scaled_mask, dst_idx);
        }
    }
    uint64_t edges[3] = {
            scaled_bits[0] ^ (scaled_bits[0] << 1),
            scaled_bits[1] ^ (scaled_bits[1] << 1),
            scaled_bits[2] ^ (scaled_bits[2] << 1),
    };
    uint8_t offset = 0;
    uint8_t lowest_bit_errors = UINT8_MAX;
    for (uint8_t i = 0; i < 3; ++i) {
        uint8_t bit_errors = 0;
        uint64_t repeating001s = 0x9249249249249249ull << i;
        for (uint8_t j = 0; j < 3; ++j) {
            bit_errors += count_bits((edges[j] ^ repeating001s) & scaled_mask[j]);
            repeating001s >>= 1;
        }
        if (bit_errors < lowest_bit_errors) {
            lowest_bit_errors = bit_errors;
            offset = i;
        }
    }
    static const uint8_t count_bits_3[8] = {0, 1, 1, 2, 1, 2, 2, 3};
    axiscode.bits = 0;
    axiscode.mask = 0;
    dst_idx -= 3;
    for (uint64_t current_bit = 1; offset < dst_idx; offset += 3, current_bit <<= 1) {
        uint8_t mask_triplet = mlsq_code_from_position((uint32_t*) scaled_mask, 3, offset);
        if (mask_triplet == 7) {
            axiscode.mask |= current_bit;
            uint8_t bit_triplet = mlsq_code_from_position((uint32_t*) scaled_bits, 3, offset);
            if (2 * count_bits_3[bit_triplet] > 3) {
                axiscode.bits |= current_bit;
            }
        }
    }
    return axiscode;
}

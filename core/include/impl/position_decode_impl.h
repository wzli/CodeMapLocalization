#ifndef WIDTH
#error "define WIDTH macro before including this header"
#endif

#include "location_decode.h"

#define TEMPLATE_CAT3(A, B, C) A##B##C
#define TEMPLATE(A, B, C) TEMPLATE_CAT3(A, B, C)

uint8_t TEMPLATE(ac, WIDTH, _next_valid_segment)(AxisCode* axiscode, uint8_t code_length) {
    assert(axiscode);
    assert(code_length <= (WIDTH));
    TEMPLATE(uint, WIDTH, _t)* bits = &TEMPLATE(axiscode->bits.x, WIDTH, );
    TEMPLATE(uint, WIDTH, _t)* mask = &TEMPLATE(axiscode->mask.x, WIDTH, );
    uint8_t valid_segment_length = count_trailing_zeros(~*mask);
    while (*mask && valid_segment_length < code_length) {
        *mask >>= valid_segment_length;
        *bits >>= valid_segment_length;
        valid_segment_length = count_trailing_zeros(*mask);
        *mask >>= valid_segment_length;
        *bits >>= valid_segment_length;
        valid_segment_length = count_trailing_zeros(~*mask);
    }
    return valid_segment_length;
}

AxisPosition TEMPLATE(ac, WIDTH, _decode_position)(AxisCode axiscode) {
    AxisPosition best_position = {0};
    TEMPLATE(uint, WIDTH, _t)* bits = &TEMPLATE(axiscode.bits.x, WIDTH, );
    const uint32_t code_mask = mask_bits(MLS_INDEX.code_length);
    for (uint8_t valid_segment_length =
                    TEMPLATE(ac, WIDTH, _next_valid_segment)(&axiscode, MLS_INDEX.code_length);
            valid_segment_length > 0;
            valid_segment_length =
                    TEMPLATE(ac, WIDTH, _next_valid_segment)(&axiscode, MLS_INDEX.code_length)) {
        uint32_t code = *bits & code_mask;
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
            int32_t lookup_index = position.center;
            if (position.reversed) {
                lookup_index += MLS_INDEX.code_length - valid_segment_length;
            }
            if (lookup_index < 0) {
                valid_segment_length += lookup_index;
                lookup_index = 0;
            }
            if (lookup_index + valid_segment_length >= MLS_INDEX.sequence_length) {
                valid_segment_length = MLS_INDEX.sequence_length - lookup_index - 1;
            }
            TEMPLATE(uint, WIDTH, _t)
            expected_code = TEMPLATE(bv32_get_slice_, WIDTH, )(
                    MLS_INDEX.sequence, lookup_index, valid_segment_length);
            TEMPLATE(uint, WIDTH, _t)
            extended_code = position.inverted
                                    ? invert_bits(*bits, valid_segment_length)
                                    : *bits & TEMPLATE(mask_bits_, WIDTH, )(valid_segment_length);
            if (position.reversed) {
                expected_code = reverse_bits(expected_code, valid_segment_length);
            }
            uint8_t first_diff = count_trailing_zeros(expected_code ^ extended_code);
            position.span += MIN(first_diff, valid_segment_length) - MLS_INDEX.code_length;
        }
        if (position.span > best_position.span) {
            best_position = position;
        }
        if (position.span >= (WIDTH)) {
            break;
        }
        position.span += !position.span;
        *bits >>= position.span;
        TEMPLATE(axiscode.mask.x, WIDTH, ) >>= position.span;
    }
    best_position.center +=
            best_position.reversed ? 1 - best_position.span / 2 : best_position.span / 2;
    if (axiscode.n_errors * 4 > axiscode.n_samples) {
        best_position.span = 0;
    }
    return best_position;
}

void TEMPLATE(ac, WIDTH, _scale_search_location)(
        ScaleMatch* match, const AxisCode* row_code, const AxisCode* col_code, float decay_rate) {
    assert(match && row_code && col_code && decay_rate > 0 && decay_rate <= 1);
    ScaleMatch candidate;
    for (candidate.scale = 1.0f; candidate.scale >= 1.0f / 3; candidate.scale *= 1 - decay_rate) {
        // scale and down sample axis codes
        candidate.row_code = *row_code;
        candidate.col_code = *col_code;
        candidate.row_scale_errors =
                TEMPLATE(ac, WIDTH, _downsample)(&candidate.row_code, candidate.scale);
        candidate.col_scale_errors =
                TEMPLATE(ac, WIDTH, _downsample)(&candidate.col_code, candidate.scale);
        // decode posiiton
        AxisPosition row_pos = TEMPLATE(ac, WIDTH, _decode_position)(candidate.row_code);
        AxisPosition col_pos = TEMPLATE(ac, WIDTH, _decode_position)(candidate.col_code);
        candidate.location = deduce_location(row_pos, col_pos);
        float scaled_bits = candidate.scale * (WIDTH);
        uint8_t scale_errors = MAX(candidate.row_scale_errors, candidate.col_scale_errors);
        candidate.quality = scaled_bits / (scaled_bits + scale_errors);
        candidate.quality /= (scaled_bits - MLS_INDEX.code_length - 1);
        candidate.quality *= candidate.quality * candidate.location.match_size;
        if (candidate.quality >= match->quality) {
            *match = candidate;
        }
    }
}

#undef WIDTH
#undef TEMPLATE_CAT3
#undef TEMPLATE

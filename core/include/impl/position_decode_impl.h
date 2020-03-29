#ifndef WIDTH
#error "define WIDTH macro before including this header"
#endif

#include "location_decode.h"
#include "mls_query.h"

#define TEMPLATE_CAT3(A, B, C) A##B##C
#define T(A, B, C) TEMPLATE_CAT3(A, B, C)

uint8_t T(ac, WIDTH, _next_valid_segment)(AxisCode* axiscode, uint8_t code_length) {
    if (code_length > (WIDTH)) {
        return 0;
    }
    assert(axiscode);
    T(uint, WIDTH, _t)* const bits = &T(axiscode->bits.x, WIDTH, );
    T(uint, WIDTH, _t)* const mask = &T(axiscode->mask.x, WIDTH, );
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

AxisPosition T(ac, WIDTH, _decode_position)(AxisCode axiscode) {
    AxisPosition best_position = {0};
    T(uint, WIDTH, _t)* const bits = &T(axiscode.bits.x, WIDTH, );
    const uint32_t code_mask = mask_bits(MLS_INDEX.code_length);
    uint8_t valid_segment_length =
            T(ac, WIDTH, _next_valid_segment)(&axiscode, MLS_INDEX.code_length);
    while (valid_segment_length > 0) {
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
            T(uint, WIDTH, _t)
            expected_code = T(bv32_get_slice_, WIDTH, )(
                    MLS_INDEX.sequence, lookup_index, valid_segment_length);
            T(uint, WIDTH, _t)
            extended_code = position.inverted
                                    ? invert_bits(*bits, valid_segment_length)
                                    : *bits & T(mask_bits_, WIDTH, )(valid_segment_length);
            if (position.reversed) {
                expected_code = reverse_bits(expected_code, valid_segment_length);
            }
            uint8_t first_diff = count_trailing_zeros(expected_code ^ extended_code);
            position.span += MIN(first_diff, valid_segment_length) - MLS_INDEX.code_length;
        }
        if (position.span > best_position.span) {
            best_position = position;
        }
        uint8_t shift = position.span > 0 ? position.span + MLS_INDEX.code_length - 1 : 1;
        *bits >>= shift;
        T(axiscode.mask.x, WIDTH, ) >>= shift;
        valid_segment_length = T(ac, WIDTH, _next_valid_segment)(
                &axiscode, MLS_INDEX.code_length + best_position.span);
    }
    best_position.center +=
            best_position.reversed ? (1 - best_position.span) / 2 : best_position.span / 2;
    return best_position;
}

void T(ac, WIDTH, _scale_search_location)(
        ScaleMatch* match, const AxisCode* row_code, const AxisCode* col_code, float decay_rate) {
    assert(match && row_code && col_code && decay_rate > 0 && decay_rate <= 1);
    ScaleMatch sample;
    uint32_t sample_count = 0;
    uint32_t error_sum = 0;
    T(uint, WIDTH, _t) prev_row_code = 0;
    T(uint, WIDTH, _t) prev_col_code = 0;
    AxisPosition row_pos;
    AxisPosition col_pos;
    for (sample.scale = 1.0f; sample.scale >= 1.0f / 3; sample.scale *= 1 - decay_rate) {
        // scale and downsample axis codes
        sample.row_code = *row_code;
        sample.col_code = *col_code;
        uint8_t row_scale_errors = T(ac, WIDTH, _downsample)(&sample.row_code, sample.scale);
        uint8_t col_scale_errors = T(ac, WIDTH, _downsample)(&sample.col_code, sample.scale);
        sample.bit_errors = row_scale_errors + col_scale_errors;
        // only proceed if scale error is better than average
        error_sum += sample.bit_errors;
        if (sample.bit_errors * (++sample_count) > error_sum) {
            continue;
        }
        // decode posiiton
        if (prev_row_code != T(sample.row_code.bits.x, WIDTH, )) {
            row_pos = T(ac, WIDTH, _decode_position)(sample.row_code);
            prev_row_code = T(sample.row_code.bits.x, WIDTH, );
        }
        if (prev_col_code != T(sample.col_code.bits.x, WIDTH, )) {
            col_pos = T(ac, WIDTH, _decode_position)(sample.col_code);
            prev_col_code = T(sample.col_code.bits.x, WIDTH, );
        }
        // calculate location and match quality
        sample.location = deduce_location(row_pos, col_pos);
        float scaled_bits = sample.scale * (WIDTH);
        sample.quality = (scaled_bits - sample.bit_errors) / scaled_bits;
        sample.quality /= (scaled_bits - MLS_INDEX.code_length - 1);
        sample.quality *= sample.quality * sample.location.match_size;
        if (sample.quality >= match->quality) {
            *match = sample;
        }
    }
    match->quality *= (1.0f - (float) row_code->n_errors / row_code->n_samples);
    match->quality *= (1.0f - (float) col_code->n_errors / col_code->n_samples);
}

#undef WIDTH
#undef TEMPLATE_CAT3
#undef T

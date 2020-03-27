#ifndef WIDTH
#error "define WIDTH macro before including this header"
#endif

#include "location_decode.h"

#define TEMPLATE_CAT3(A, B, C) A##B##C
#define TEMPLATE(A, B, C) TEMPLATE_CAT3(A, B, C)

uint8_t TEMPLATE(ac, WIDTH, _next_valid_segment)(
        TEMPLATE(AxisCode, WIDTH, ) * axiscode, uint8_t code_length) {
    assert(axiscode);
    assert(code_length <= (WIDTH));
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

AxisPosition TEMPLATE(ac, WIDTH, _decode_position)(TEMPLATE(AxisCode, WIDTH, ) axiscode) {
    AxisPosition best_position = {0};
    const uint32_t code_mask = mask_bits(MLS_INDEX.code_length);
    for (uint8_t valid_segment_length =
                    TEMPLATE(ac, WIDTH, _next_valid_segment)(&axiscode, MLS_INDEX.code_length);
            valid_segment_length > 0;
            valid_segment_length =
                    TEMPLATE(ac, WIDTH, _next_valid_segment)(&axiscode, MLS_INDEX.code_length)) {
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
            TEMPLATE(uint, WIDTH, _t)
            extended_code = position.inverted ? invert_bits(axiscode.bits, valid_segment_length)
                                              : axiscode.bits & TEMPLATE(mask_bits_, WIDTH, )(
                                                                        valid_segment_length);
            uint32_t lookup_index = position.reversed ? position.center + MLS_INDEX.code_length -
                                                                valid_segment_length
                                                      : position.center;
            TEMPLATE(uint, WIDTH, _t)
            expected_code = lookup_index + valid_segment_length >= MLS_INDEX.sequence_length
                                    ? 0
                                    : TEMPLATE(bv32_get_slice_, WIDTH, )(MLS_INDEX.sequence,
                                              lookup_index, valid_segment_length);
            if (position.reversed) {
                expected_code = reverse_bits(expected_code, valid_segment_length);
            }
            uint8_t first_diff = count_trailing_zeros(extended_code ^ expected_code);
            position.span += MIN(first_diff, valid_segment_length) - MLS_INDEX.code_length;
        }
        if (position.span > best_position.span) {
            best_position = position;
        }
        if (position.span >= (WIDTH)) {
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

#undef WIDTH
#undef TEMPLATE_CAT3
#undef TEMPLATE

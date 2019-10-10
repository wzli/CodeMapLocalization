#include "image_filter/code_map_filter.h"
#include "mls_query/mls_query.h"

typedef enum {
    CODE_VERDICT_DIRECT,
    CODE_VERDICT_INVERSE,
    CODE_VERDICT_REVERSE,
    CODE_VERDICT_INVERSE_REVERSE,
    CODE_VERDICT_INSUFFICIENT_BITS,
} CodeVerdict;

uint8_t skip_to_valid_code_segment(AxisCode* axis_code, uint8_t code_length) {
    if (axis_code->mask == ~0) {
        return 32;
    }
    uint8_t valid_segment_length = first_set_bit(~axis_code->mask);
    while (valid_segment_length > 0 && valid_segment_length < code_length) {
        axis_code->mask >>= valid_segment_length;
        axis_code->bits >>= valid_segment_length;
        valid_segment_length = first_set_bit(~axis_code->mask);
    }
    return valid_segment_length;
};

CodeVerdict decode_axis(uint16_t* output_position, AxisCode axis_code, uint8_t code_length) {
    uint32_t code_mask = ~(~0 << code_length);
    uint32_t code, position;
    uint8_t valid_segment_length = skip_to_valid_code_segment(&axis_code, code_length);
    CodeVerdict best_position_verdict = CODE_VERDICT_INSUFFICIENT_BITS;
    uint16_t best_position = MLSQ_NOT_FOUND;
    uint8_t min_position_errors = ~0;
    while (min_position_errors > 0 && valid_segment_length > 0) {
        CodeVerdict verdict = CODE_VERDICT_DIRECT;
        code = axis_code.bits & code_mask;
        position = mlsq_position_from_code(MLSQ_INDEX, code);

        if (position == MLSQ_NOT_FOUND) {
            verdict = CODE_VERDICT_INVERSE;
            code = inverse_bits(code, code_length);
            position = mlsq_position_from_code(MLSQ_INDEX, code);
        }

        if (position == MLSQ_NOT_FOUND) {
            verdict = CODE_VERDICT_REVERSE;
            code = reverse_bits(code, code_length);
            position = mlsq_position_from_code(MLSQ_INDEX, code);
        }

        if (position == MLSQ_NOT_FOUND) {
            verdict = CODE_VERDICT_INVERSE_REVERSE;
            code = inverse_bits(code, code_length);
            position = mlsq_position_from_code(MLSQ_INDEX, code);
        }

        if (position == MLSQ_NOT_FOUND) {
            axis_code.bits >>= 1;
            axis_code.mask >>= 1;
            valid_segment_length = skip_to_valid_code_segment(&axis_code, code_length);
            continue;
        }

        uint32_t extended_code = axis_code.bits & ~(~0 << valid_segment_length);
        uint32_t expected_code;
        switch (verdict) {
            case CODE_VERDICT_INVERSE:
                extended_code = inverse_bits(extended_code, valid_segment_length);
            case CODE_VERDICT_DIRECT:
                expected_code = mlsq_code_from_position(
                        MLSQ_INDEX.sequence, valid_segment_length, position);
                break;
            case CODE_VERDICT_INVERSE_REVERSE:
                extended_code = inverse_bits(extended_code, valid_segment_length);
            case CODE_VERDICT_REVERSE:
                extended_code = reverse_bits(extended_code, valid_segment_length);
                expected_code = mlsq_code_from_position(MLSQ_INDEX.sequence, valid_segment_length,
                        position + code_length - valid_segment_length);
            default:
                break;
        }
        uint8_t position_errors = bit_sum(extended_code ^ expected_code);
        if (position_errors < min_position_errors) {
            min_position_errors = position_errors;
            best_position_verdict = verdict;
            best_position = position;
        }
    }
    if (!valid_segment_length) {
        return CODE_VERDICT_INSUFFICIENT_BITS;
    }
    *output_position = best_position;
    return best_position_verdict;
}

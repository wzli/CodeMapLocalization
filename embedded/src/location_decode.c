#include "location_decode.h"
#include "mls_query.h"
#include <assert.h>

#include "test_utils.h"

uint8_t next_valid_code_segment(AxisCode* axis_code, uint8_t code_length) {
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

CodeVerdict decode_axis(uint16_t* output_position, AxisCode axis_code, uint8_t code_length) {
    *output_position = MLSQ_NOT_FOUND;
    CodeVerdict best_verdict = CODE_VERDICT_ERROR;
    uint8_t max_position_matches = 0;
    uint32_t code_mask = mask_bits(code_length);
    uint8_t valid_segment_length = next_valid_code_segment(&axis_code, code_length);
    while (valid_segment_length > 0 &&
            max_position_matches <= (valid_segment_length - code_length)) {
        CodeVerdict verdict = CODE_VERDICT_DIRECT;
        uint32_t code = axis_code.bits & code_mask;
        uint16_t position = mlsq_position_from_code(MLSQ_INDEX, code);

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
            valid_segment_length = next_valid_code_segment(&axis_code, code_length);
            continue;
        }

        uint32_t extended_code = axis_code.bits;
        extended_code &= mask_bits(valid_segment_length);
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
                break;
            default:
                assert(0);
        }
        uint8_t position_matches =
                valid_segment_length - code_length + 1 - sum_bits(extended_code ^ expected_code);
        if (position_matches > max_position_matches) {
            max_position_matches = position_matches;
            *output_position = position;
            best_verdict = verdict;
        }
    }
    if (!valid_segment_length) {
        return CODE_VERDICT_ERROR;
    }
    return best_verdict;
}

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

#undef WIDTH
#undef TEMPLATE_CAT3
#undef TEMPLATE

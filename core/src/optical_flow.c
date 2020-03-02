#include "optical_flow.h"
#include <assert.h>

void img_phase_correlation(
        ImageMatrixComplex frame, ImageMatrixComplex next_frame, bool reuse_frame) {
    assert(next_frame.size.x >= 0 && next_frame.size.y >= 0);
    assert(frame.size.x == next_frame.size.x && frame.size.y == next_frame.size.y);
    if (!reuse_frame) {
        img_fast_fourier_transform(frame, false);
    }
    img_fast_fourier_transform(next_frame, false);
    FOR_EACH_PIXEL(frame) {
        Vector2f* a = (Vector2f*) &PIXEL(frame, row, col);
        Vector2f* b = (Vector2f*) &PIXEL(next_frame, row, col);
        Vector2f c = {v2f_dot(*b, *a), v2f_cross(*b, *a)};
        *a = v2f_normalize(c);
    }
    img_fast_fourier_transform(frame, false);
}

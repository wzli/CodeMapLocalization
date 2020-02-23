#include "image_utils.h"
#include <assert.h>

static ImageMatrixComplex img_convert_int32_to_complex(ImageMatrixInt32 frame) {
    ImageMatrixComplex complex_frame = {(float complex*) frame.data, frame.size};
    FOR_EACH_PIXEL(frame) {
        int16_t r_row = frame.size.y - row - 1;
        int16_t r_col = frame.size.x - col - 1;
        PIXEL(complex_frame, r_row, r_col) = PIXEL(frame, r_row, r_col);
    }
    return complex_frame;
};

void img_phase_correlation(ImageMatrixInt32 frame, ImageMatrixInt32 next_frame, bool reuse_frame) {
    assert(next_frame.size.x >= 0 && next_frame.size.y >= 0);
    assert(frame.size.x == next_frame.size.x && frame.size.y == next_frame.size.y);

    ImageMatrixComplex complex_frame;
    if (reuse_frame) {
        complex_frame = (ImageMatrixComplex){(float complex*) frame.data, frame.size};
    } else {
        complex_frame = img_convert_int32_to_complex(frame);
        img_fast_fourier_transform(complex_frame);
    }

    ImageMatrixComplex next_complex_frame = img_convert_int32_to_complex(next_frame);
    img_fast_fourier_transform(next_complex_frame);

    FOR_EACH_PIXEL(frame) {
        PIXEL(complex_frame, row, col) *= conjf(PIXEL(next_complex_frame, row, col));
        PIXEL(complex_frame, row, col) /= cabsf(PIXEL(complex_frame, row, col));
    }
    img_inverse_fast_fourier_transform(complex_frame);

    FOR_EACH_PIXEL(frame) {
        PIXEL(frame, row, col) = crealf(PIXEL(complex_frame, row, col)) + 0.5f;
    }
}

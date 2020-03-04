#include "visual_odometry.h"
#include <assert.h>

// hann window f(i) = cos(pi * (i - 31.5) / 44) ^ 2
static const float window_lookup[64] = {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.00127394f,
        0.01142657f,
        0.03152514f,
        0.06116051f,
        0.09972938f,
        0.14644661f,
        0.20036117f,
        0.26037551f,
        0.32526791f,
        0.39371736f,
        0.46433041f,
        0.53566959f,
        0.60628264f,
        0.67473209f,
        0.73962449f,
        0.79963883f,
        0.85355339f,
        0.90027062f,
        0.93883949f,
        0.96847486f,
        0.98857343f,
        0.99872606f,
        0.99872606f,
        0.98857343f,
        0.96847486f,
        0.93883949f,
        0.90027062f,
        0.85355339f,
        0.79963883f,
        0.73962449f,
        0.67473209f,
        0.60628264f,
        0.53566959f,
        0.46433041f,
        0.39371736f,
        0.32526791f,
        0.26037551f,
        0.20036117f,
        0.14644661f,
        0.09972938f,
        0.06116051f,
        0.03152514f,
        0.01142657f,
        0.00127394f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
};

Vector2f img_estimate_rotation(const ImageMatrix mat) {
    Vector2f gradient_sum = {};
    ImageMatrix bounds = {0, {mat.size.x - 2, mat.size.y - 2}};
    FOR_EACH_PIXEL(bounds) {
        Vector2f gradient = {{
                PIXEL(mat, row, col + 2) - PIXEL(mat, row, col) +
                        2 * (PIXEL(mat, row + 1, col + 2) - PIXEL(mat, row + 1, col)) +
                        PIXEL(mat, row + 2, col + 2) - PIXEL(mat, row + 2, col),
                PIXEL(mat, row + 2, col) - PIXEL(mat, row, col) +
                        2 * (PIXEL(mat, row + 2, col + 1) - PIXEL(mat, row, col + 1)) +
                        PIXEL(mat, row + 2, col + 2) - PIXEL(mat, row, col + 2),
        }};
        gradient.z *= gradient.z;
        gradient.z *= gradient.z;
        gradient_sum.z += gradient.z;
    }
    if (gradient_sum.z != 0) {
        gradient_sum = v2f_normalize(gradient_sum);
        gradient_sum.z = csqrtf(csqrtf(gradient_sum.z));
        assert(!v2f_is_nan(gradient_sum));
    }
    return gradient_sum;
}

void img_estimate_translation(Correlation* correlation, const ImageMatrix frame) {
    assert(frame.size.x == 64 && frame.size.y == 64);
    IMG_SET_SIZE(correlation->image, 32, 32);
    IMG_SET_SIZE(correlation->buffer, 32, 32);
    // 2x2 bin the source image
    IMG_FILL(correlation->image, (Vector2f){});
    FOR_EACH_PIXEL(frame) {
        // apply hann window to remove edge effects
        PIXEL(correlation->image, row / 2, col / 2).x +=
                PIXEL(frame, row, col) * window_lookup[row] * window_lookup[col];
    }
    // reuse previously transfromed image as 1st frame
    SWAP(correlation->image.data, correlation->buffer.data);
    img_phase_correlation(correlation->image, correlation->buffer, true);
    // find peak value in correlation image
    correlation->max_squared_magnitude = 0;
    FOR_EACH_PIXEL(correlation->image) {
        float mag_sqr = v2f_norm_sqr(PIXEL(correlation->image, row, col));
        PIXEL(correlation->image, row, col).z = mag_sqr;
        if (correlation->max_squared_magnitude < mag_sqr) {
            correlation->max_squared_magnitude = mag_sqr;
            correlation->translation.z = col + I * row;
        }
    }
    if (correlation->translation.x > 16) {
        correlation->translation.x -= 32;
    }
    if (correlation->translation.y > 16) {
        correlation->translation.y -= 32;
    }
    correlation->translation.z *= 2;
}

void img_phase_correlation(
        ImageMatrixComplex frame, ImageMatrixComplex next_frame, bool reuse_frame) {
    assert(next_frame.size.x >= 0 && next_frame.size.y >= 0);
    assert(frame.size.x == next_frame.size.x && frame.size.y == next_frame.size.y);
    if (!reuse_frame) {
        img_fast_fourier_transform(frame, false);
    }
    img_fast_fourier_transform(next_frame, false);
    FOR_EACH_PIXEL(frame) {
        PIXEL(frame, row, col).z *= conjf(PIXEL(next_frame, row, col).z);
        PIXEL(frame, row, col) = v2f_normalize(PIXEL(frame, row, col));
    }
    img_fast_fourier_transform(frame, false);
}

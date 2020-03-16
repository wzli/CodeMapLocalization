#include "visual_odometry.h"
#include <assert.h>

#define CORRELATION_SIZE (64)

// hann window f(i) = cos(pi * (i - 31.5) / 44) ^ 2
static const float WINDOW_LOOKUP[CORRELATION_SIZE] = {
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
    ImageMatrix bounds = {0, {{mat.size.x - 2, mat.size.y - 2}}};
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

void odom_update(OdometryContext* ctx, ImageMatrix image, Vector2f rotation, float scale) {
    // track if quadrants changed
    int8_t quadrant_increment = odom_track_rotation(ctx, rotation);
    // rotate filter out quadrant changes
    if (quadrant_increment != 0) {
        Vector2f fill = {};
        IMG_FILL(ctx->correlation.buffer, fill);
    }
    img_estimate_translation(&(ctx->correlation), image);
    ctx->position.z +=
            ctx->correlation.translation.z * QUADRANT_LOOKUP[ctx->quadrant_count & 3].z * scale;
}

void odom_set_location(OdometryContext* ctx, Location loc) {
    ctx->position.x = loc.x;
    ctx->position.y = loc.y;
    ctx->quadrant_count &= ~3u;
    ctx->quadrant_count |= loc.direction;
}

int8_t odom_track_rotation(OdometryContext* ctx, Vector2f quadrant_rotation) {
    assert(ctx);
    float dy = quadrant_rotation.y - ctx->quadrant_rotation.y;
    ctx->quadrant_rotation = quadrant_rotation;
    if (dy < -0.8f) {
        ++ctx->quadrant_count;
        return 1;
    }
    if (dy > 0.8f) {
        --ctx->quadrant_count;
        return -1;
    }
    return 0;
}

void img_estimate_translation(Correlation* correlation, const ImageMatrix frame) {
    assert(frame.size.x == CORRELATION_SIZE && frame.size.y == CORRELATION_SIZE);
    IMG_SET_SIZE(correlation->image, CORRELATION_SIZE / 2, CORRELATION_SIZE / 2);
    IMG_SET_SIZE(correlation->buffer, CORRELATION_SIZE / 2, CORRELATION_SIZE / 2);
    // 2x2 bin the source image
    IMG_FILL(correlation->image, (Vector2f){});
    FOR_EACH_PIXEL(frame) {
        // apply hann window to remove edge effects
        PIXEL(correlation->image, row / 2, col / 2).x +=
                PIXEL(frame, row, col) * WINDOW_LOOKUP[row] * WINDOW_LOOKUP[col];
    }
    // reuse previously transfromed image as 1st frame
    SWAP(correlation->image.data, correlation->buffer.data);
    img_phase_correlation(correlation->image, correlation->buffer, true);
    // find peak value in correlation image
    correlation->max_squared_magnitude = 0;
    correlation->translation.z = 0;
    FOR_EACH_PIXEL(correlation->image) {
        float mag_sqr = v2f_norm_sqr(PIXEL(correlation->image, row, col));
        PIXEL(correlation->image, row, col).z = mag_sqr;
        if (correlation->max_squared_magnitude <= mag_sqr) {
            correlation->max_squared_magnitude = mag_sqr;
            correlation->translation.z = col + I * row;
        }
    }
    // return early if correlation is empty
    if (correlation->max_squared_magnitude == 0) {
        correlation->translation.z = 0;
        return;
    }
    // find subpixel shift
    Vector2f subpixel_shift = img_subpixel_registration(correlation);
    // zero center the peak index
    for (uint8_t i = 0; i < 2; ++i) {
        if (correlation->translation.data[i] > (CORRELATION_SIZE / 4)) {
            correlation->translation.data[i] -= (CORRELATION_SIZE / 2);
        }
    }
    correlation->translation.z += subpixel_shift.z;
    correlation->translation.z *= -2;
}

Vector2f img_subpixel_registration(const Correlation* correlation) {
    assert(correlation && correlation->max_squared_magnitude > 0);
    // initialize peaks (assume correlation image is square normalized)
    const float peak = sqrtf(correlation->max_squared_magnitude);
    int16_t peak_idx[2] = {
            (int16_t) correlation->translation.x, (int16_t) correlation->translation.y};
    int8_t side_peaks_dir[2] = {1, 1};
    static const uint8_t mod_mask = (CORRELATION_SIZE / 2) - 1;
    Vector2f side_peaks = {{
            PIXEL(correlation->image, peak_idx[1], (peak_idx[0] + 1) & mod_mask).x,
            PIXEL(correlation->image, (peak_idx[1] + 1) & mod_mask, peak_idx[0]).x,
    }};
    Vector2f other_side_peaks = {{
            PIXEL(correlation->image, peak_idx[1], (peak_idx[0] - 1) & mod_mask).x,
            PIXEL(correlation->image, (peak_idx[1] - 1) & mod_mask, peak_idx[0]).x,
    }};
    Vector2f subpixel_shift;
    for (uint8_t i = 0; i < 2; ++i) {
        // find the highest side peak
        if (side_peaks.data[i] < other_side_peaks.data[i]) {
            side_peaks.data[i] = other_side_peaks.data[i];
            side_peaks_dir[i] = -1;
        }
        // compute subpixel shift
        side_peaks.data[i] = sqrtf(side_peaks.data[i]);
        subpixel_shift.data[i] =
                side_peaks_dir[i] * side_peaks.data[i] / (side_peaks.data[i] + peak);
    };
    return subpixel_shift;
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

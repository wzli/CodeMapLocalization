#include "image_utils.h"
#include "esp_dsp.h"
#include <assert.h>

static esp_err_t dsp_fft(int16_t* data, int len, bool inverse) {
    const int16_t* const end = data + 2 * len;
    for (int16_t* cur = data + 1; inverse && cur < end; *cur = -*cur, cur += 2)
        ;
    esp_err_t esp_error = dsps_fft2r_sc16(data, len);
    for (int16_t* cur = data + 1; inverse && cur < end; *cur = -*cur, cur += 2)
        ;
    esp_error |= dsps_bit_rev_sc16_ansi(data, len);
    return esp_error;
}

static esp_err_t dsp_fft_2d(ImageMatrixInt32 frame, bool inverse) {
    assert(frame.size.x == frame.size.y);
    esp_err_t esp_error = ESP_OK;
    IMG_TRANSPOSE(frame, frame);
    for (int x = 0; x < frame.size.x; ++x, frame.data += frame.size.y) {
        esp_error |= dsp_fft((int16_t*) frame.data, frame.size.y, inverse);
    }
    frame.data -= IMG_PIXEL_COUNT(frame);
    IMG_TRANSPOSE(frame, frame);
    for (int y = 0; y < frame.size.y; ++y, frame.data += frame.size.x) {
        esp_error |= dsp_fft((int16_t*) frame.data, frame.size.x, inverse);
    }
    return esp_error;
}

void img_phase_correlation(ImageMatrixInt32 frame, ImageMatrixInt32 next_frame, bool reuse_frame) {
    assert(frame.data && next_frame.data);
    assert(frame.size.x >= 0 && frame.size.y >= 0);
    assert(frame.size.x == next_frame.size.x);
    assert(frame.size.y == next_frame.size.y);
    esp_err_t esp_error = ESP_OK;
    // FFT frame
    if (!reuse_frame) {
        esp_error |= dsp_fft_2d(frame, false);
    }
    // FFT next frame
    esp_error |= dsp_fft_2d(next_frame, false);
    // Element wise multiply and normalize
    int16_t* a = (int16_t*) frame.data;
    int16_t* b = (int16_t*) next_frame.data;
    for (int32_t i = 0; i < IMG_PIXEL_COUNT(frame); ++i, a += 2, b += 2) {
        Vector2f c = {(a[0] * b[0]) + (a[1] * b[1]), (b[0] * a[1]) - (a[0] * b[1])};
        c = v2f_normalize(c);
        a[0] = 0x7FFF * c.x;
        a[1] = 0x7FFF * c.y;
    }
    // Inverse FFT
    esp_error |= dsp_fft_2d(frame, true);
    // Only keep magnitude
    a = (int16_t*) frame.data;
    for (int32_t i = 0; i < IMG_PIXEL_COUNT(frame); ++i, a += 2) {
        frame.data[i] = (SQR(a[0]) + SQR(a[1])) / 128;
    }
    assert(esp_error == ESP_OK);
}

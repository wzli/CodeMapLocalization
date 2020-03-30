#include "image_utils.h"
#include "esp_dsp.h"
#include <assert.h>

// internally call the optimized FFT implementation from esp-dsp

static esp_err_t dsp_fft(float* data, int len, bool inverse) {
    const float* const end = data + 2 * len;
    for (float* cur = data + 1; inverse && cur < end; *cur = -*cur, cur += 2)
        ;
    esp_err_t esp_error = dsps_fft2r_fc32(data, len);
    for (float* cur = data; inverse && cur < end; cur[0] /= len, cur[1] /= -len, cur += 2)
        ;
    esp_error |= dsps_bit_rev_fc32(data, len);
    return esp_error;
}

static esp_err_t dsp_fft_2d(ImageMatrixComplex frame, bool inverse) {
    assert(IMG_IS_VALID(frame) && frame.size.x == frame.size.y);
    esp_err_t esp_error = ESP_OK;
    IMG_TRANSPOSE(frame, frame);
    for (int x = 0; x < frame.size.x; ++x, frame.data += frame.size.y) {
        esp_error |= dsp_fft((float*) frame.data, frame.size.y, inverse);
    }
    frame.data -= IMG_PIXEL_COUNT(frame);
    IMG_TRANSPOSE(frame, frame);
    for (int y = 0; y < frame.size.y; ++y, frame.data += frame.size.x) {
        esp_error |= dsp_fft((float*) frame.data, frame.size.x, inverse);
    }
    return esp_error;
}

// implement the mfft interface

void fast_fourier_transform(float complex* data, int len, int stride, bool inverse) {
    assert(stride == 1);
    esp_err_t esp_error = dsp_fft((float*) data, len, inverse);
    assert(esp_error == ESP_OK);
}

void fft_2d(float complex* data, int x_len, int y_len, bool inverse) {
    ImageMatrixComplex image = {(Vector2f*) data, {x_len, y_len}};
    esp_err_t esp_error = dsp_fft_2d(image, inverse);
    assert(esp_error == ESP_OK);
}

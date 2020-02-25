#include "image_utils.h"
#include "assert.h"
#include "esp_dsp.h"
#include "esp_log.h"

typedef union {
    int16_t half[2];
    int32_t full;
} Word;

#define RE(X) ((Word)(X)).half[0]
#define IM(X) ((Word)(X)).half[1]

static const char* TAG = "phase_correlation";

static esp_err_t dsp_fft(int16_t* data, int len, bool inverse) {
    if (inverse) {
        for (int i = 0; i < len; ++i) {
            data[2 * i + 1] *= -1;
        }
    }
    esp_err_t esp_error = dsps_fft2r_sc16(data, len);
    if (inverse) {
        for (int i = 0; i < len; ++i) {
            data[2 * i] /= len;
            data[2 * i + 1] /= -len;
        }
    }
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

static int32_t complex_conjugate(int32_t x) {
    IM(x) = -IM(x);
    return x;
}

static int32_t complex_multiply(int32_t a, int32_t b) {
    return (Word){{(RE(a) * RE(b)) - (IM(a) * IM(b)), (RE(a) * IM(b)) + (RE(b) * IM(a))}}.full;
}

static float complex_norm(int32_t x) {
    return sqrtf(SQR(RE(x)) + SQR(IM(x)));
}

void img_phase_correlation(ImageMatrixInt32 frame, ImageMatrixInt32 next_frame, bool reuse_frame) {
    assert(next_frame.size.x >= 0 && next_frame.size.y >= 0);
    assert(frame.size.x == next_frame.size.x && frame.size.y == next_frame.size.y);

    esp_err_t ret = dsps_fft2r_init_sc16(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
        return;
    }
    if (!reuse_frame) {
        dsp_fft_2d(frame, false);
    }
    dsp_fft_2d(next_frame, false);

    FOR_EACH_PIXEL(frame) {
        int32_t conj = complex_conjugate(PIXEL(next_frame, row, col));
        PIXEL(frame, row, col) = complex_multiply(PIXEL(frame, row, col), conj);
        float norm = complex_norm(PIXEL(frame, row, col));
        RE(PIXEL(frame, row, col)) /= norm;
        IM(PIXEL(frame, row, col)) /= norm;
    }

    dsp_fft_2d(next_frame, true);

    FOR_EACH_PIXEL(frame) { IM(PIXEL(frame, row, col)) = 0; }
}

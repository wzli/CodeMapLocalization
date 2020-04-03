#include "sdkconfig.h"
#include "esp_camera.h"
#include "esp_dsp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "tests.h"
#include "global.h"

/* defines */
#define STATIC_FRAME_BUFFER(SIZE) \
    (camera_fb_t) { (uint8_t[SQR(SIZE)]){}, SQR(SIZE), SIZE, SIZE, PIXFORMAT_GRAYSCALE }

#define STATIC_DOUBLE_BUFFER(SIZE) \
    { STATIC_FRAME_BUFFER(SIZE), STATIC_FRAME_BUFFER(SIZE) }

#define N_DOUBLE_BUFFERS (sizeof(double_buffers) / sizeof(double_buffers[0]))

/* global data */
LocalizationContext loc_ctx;

/* static data */
static const char* TAG = "main_loop";

static camera_fb_t double_buffers[][2] = {
        STATIC_DOUBLE_BUFFER(64),
        STATIC_DOUBLE_BUFFER(64),
        STATIC_DOUBLE_BUFFER(64),
};

static camera_fb_t* claimed_buffers[N_DOUBLE_BUFFERS + 1];
static ImageMatrix images[N_DOUBLE_BUFFERS + 1];

static Vector2f correlation_buffers[2][32 * 32];

/* helper functions */

static inline ImageMatrix fb_to_img(camera_fb_t fb) {
    return (ImageMatrix){fb.buf, {fb.width, fb.height}};
}

static inline camera_fb_t* camera_fb_swap(camera_fb_t* fb) {
    assert(fb);
    esp_camera_fb_return(fb);
    fb = esp_camera_fb_get();
    while (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        fb = esp_camera_fb_get();
    }
    return fb;
}

static ImageMatrix queue_fb_get(uint8_t queue_index) {
    assert(queue_index <= N_DOUBLE_BUFFERS);
    assert(!claimed_buffers[queue_index]);
    xQueueReceive(frame_queues[queue_index], &claimed_buffers[queue_index], 0);
    assert(claimed_buffers[queue_index]);
    if (queue_index == 0) {
        claimed_buffers[0] = camera_fb_swap(claimed_buffers[0]);
    }
    return fb_to_img(*claimed_buffers[queue_index]);
}

static void queue_fb_return(uint8_t queue_index) {
    assert(queue_index <= N_DOUBLE_BUFFERS);
    assert(claimed_buffers[queue_index]);
    if (pdTRUE == xQueueSendToBack(frame_queues[queue_index], &claimed_buffers[queue_index], 0)) {
        claimed_buffers[queue_index] = NULL;
    }
    assert(!claimed_buffers[queue_index]);
}

/* run */
static void main_loop(void* pvParameters) {
    // configure location context params
    loc_ctx.scale_step = 0.015f;
    loc_ctx.outlier_filter.quality_threshold = 0.05f;
    loc_ctx.outlier_filter.distance_threshold = 200;
    loc_ctx.outlier_filter.match_length_threshold = 3;
    loc_ctx.outlier_filter.xor_error_ratio_threshold = 4;
    loc_ctx.outlier_filter.max_rejection_count = 10;
    loc_ctx.odom.correlation.squared_magnitude_threshold = 0.01f;
    loc_ctx.odom.correlation.image.data = correlation_buffers[0];
    loc_ctx.odom.correlation.buffer.data = correlation_buffers[1];
    // initialize queues
    for (int i = 0; i <= N_DOUBLE_BUFFERS; ++i) {
        for (int j = 0; j < 2; ++j) {
            camera_fb_t* fb_ptr = i ? &double_buffers[i - 1][j] : esp_camera_fb_get();
            xQueueSendToBack(frame_queues[i], &fb_ptr, 0);
        }
        assert(!uxQueueSpacesAvailable(frame_queues[i]));
    }
    // main loop
    for (;;) {
        // fetch frame buffers
        for (uint8_t i = 0; i <= N_DOUBLE_BUFFERS; ++i) {
            images[i] = queue_fb_get(i);
        }
        // record time when new buffer is received
        int64_t frame_time = esp_timer_get_time();
        // start recording raw image
        if (record_task) {
            xTaskNotify(record_task, (uint32_t) claimed_buffers[0], eSetValueWithOverwrite);
        }
        // assign buffers to localization context
        IMG_SET_SIZE(images[1], 64, 64);
        loc_ctx.derotated_image = images[1];
        loc_ctx.sharpened_image = images[1];

        // run localization logic
        localization_loop_run(&loc_ctx, images[0]);
        IMG_SET_SIZE(images[1], 62, 62);

        // return frame buffers
        for (uint8_t i = 0; i <= N_DOUBLE_BUFFERS; ++i) {
            assert(claimed_buffers[i]);
            claimed_buffers[i]->width = images[i].size.x;
            claimed_buffers[i]->height = images[i].size.y;
            claimed_buffers[i]->len = IMG_PIXEL_COUNT(images[i]);
            queue_fb_return(i);
        }
        // record csv log
        if (csv_log_task) {
            xTaskNotify(csv_log_task, frame_time >> 10, eSetValueWithOverwrite);
        }
    }
}

static void side_loop(void* pvParameters) {
    uint16_t led_duty_control = 0;
    // print csv header
    while (true) {
#if 0
        // write decoded image to buffer
        bm64_from_axiscodes(loc_ctx.binary_image, loc_ctx.binary_mask,
                &loc_ctx.scale_match.row_code, &loc_ctx.scale_match.col_code);
        bm64_to_img(&images[2], loc_ctx.binary_image, loc_ctx.binary_mask);

        // write extracted image to buffer
        bm64_from_axiscodes(
                loc_ctx.binary_image, loc_ctx.binary_mask, &loc_ctx.row_code, &loc_ctx.col_code);
        bm64_to_img(&images[3], loc_ctx.binary_image, loc_ctx.binary_mask);
#endif

#if CONFIG_LED_ILLUMINATOR_ENABLED
        // control LED based on desired threshold
        if (led_duty_control < ((1 << 12) - 1) && loc_ctx.otsu_threshold < led_duty - 8) {
            set_led_duty((++led_duty_control) >> 4);
        } else if (led_duty_control > 0 && loc_ctx.otsu_threshold > led_duty + 8) {
            set_led_duty((--led_duty_control) >> 4);
        }
#endif
    }
}

void app_main() {
    // initalize FFT tables
    if (dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE) != ESP_OK) {
        ESP_LOGE(TAG, "Not able to initialize FFT.");
        assert(0);
    }

    // print MLS version
    ESP_LOGI(TAG, "Using MLS_ID %llx sequence_length %d code_length %d", MLS_ID,
            MLS_INDEX.sequence_length, MLS_INDEX.code_length);

    // run unit tests
    assert(!run_all_tests());

    // init tasks
    app_camera_main();  // camera has to start first otherwise it crashes
    app_record_main();
    app_wifi_main();
    app_httpd_main();

    // start main loops
    xTaskCreatePinnedToCore(main_loop, "main_loop", 2048, NULL, 9, NULL, 1);
}

#include "sdkconfig.h"
#include "esp_camera.h"
#include "esp_dsp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "tests.h"
#include "localization_loop.h"

/* externals */
void app_camera_main();
void app_wifi_main();
void app_httpd_main();

extern QueueHandle_t frame_queues[];

/* static data */
static const char* TAG = "main_loop";

#define STATIC_FRAME_BUFFER(SIZE) \
    (camera_fb_t) { (uint8_t[SQR(SIZE)]){}, SQR(SIZE), SIZE, SIZE, PIXFORMAT_GRAYSCALE }

#define STATIC_DOUBLE_BUFFER(SIZE) \
    { STATIC_FRAME_BUFFER(SIZE), STATIC_FRAME_BUFFER(SIZE) }

static camera_fb_t double_buffers[][2] = {
        STATIC_DOUBLE_BUFFER(64),
        STATIC_DOUBLE_BUFFER(64),
        STATIC_DOUBLE_BUFFER(64),
};

#define N_DOUBLE_BUFFERS (sizeof(double_buffers) / sizeof(double_buffers[0]))
static camera_fb_t* claimed_buffers[N_DOUBLE_BUFFERS + 1];

static LocalizationContext loc_ctx;

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
    // initalize FFT tables
    if (dsps_fft2r_init_sc16(NULL, CONFIG_DSP_MAX_FFT_SIZE) != ESP_OK) {
        ESP_LOGE(TAG, "Not able to initialize FFT.");
        assert(0);
    }
    // run unit tests
    assert(!run_all_tests());
    // configure scale search params
    loc_ctx.scale_query.lower_bound = 0.8f;
    loc_ctx.scale_query.upper_bound = 1.2f;
    loc_ctx.scale_query.step_size = 0.02f;
    // initialize queues
    for (int i = 0; i <= N_DOUBLE_BUFFERS; ++i) {
        for (int j = 0; j < 2; ++j) {
            camera_fb_t* fb_ptr = i ? &double_buffers[i - 1][j] : esp_camera_fb_get();
            xQueueSendToBack(frame_queues[i], &fb_ptr, 0);
        }
        assert(!uxQueueSpacesAvailable(frame_queues[i]));
    }
    int64_t start_time = esp_timer_get_time();
    // main loop
    for (uint32_t frame_count = 0;; ++frame_count) {
        ImageMatrix images[N_DOUBLE_BUFFERS + 1];
        for (uint8_t i = 0; i <= N_DOUBLE_BUFFERS; ++i) {
            images[i] = queue_fb_get(i);
        }
        loc_ctx.original_image = images[0];
        loc_ctx.unrotated_image = images[1];
        loc_ctx.sharpened_image = images[1];
        localization_loop_run(&loc_ctx);

        // display thresholded bm64
        bm64_to_img(&images[1], loc_ctx.binary_image, loc_ctx.binary_mask);

        // display extracted bm64
        bm64_from_axiscodes(loc_ctx.binary_image, loc_ctx.binary_mask, loc_ctx.scale_query.row_code,
                loc_ctx.scale_query.col_code);
        bm64_to_img(&images[2], loc_ctx.binary_image, loc_ctx.binary_mask);

        // display scale matched bm64
        BM_COPY(loc_ctx.scale_query.row_code, loc_ctx.scale_match.row_code);
        BM_COPY(loc_ctx.scale_query.col_code, loc_ctx.scale_match.col_code);
        bm64_from_axiscodes(loc_ctx.binary_image, loc_ctx.binary_mask, loc_ctx.scale_query.row_code,
                loc_ctx.scale_query.col_code);
        bm64_to_img(&images[3], loc_ctx.binary_image, loc_ctx.binary_mask);

        for (uint8_t i = 0; i <= N_DOUBLE_BUFFERS; ++i) {
            assert(claimed_buffers[i]);
            claimed_buffers[i]->width = images[i].size.x;
            claimed_buffers[i]->height = images[i].size.y;
            claimed_buffers[i]->len = IMG_PIXEL_COUNT(images[i]);
            queue_fb_return(i);
        }

        // end loop
        int64_t end_time = esp_timer_get_time();
        if (loc_ctx.scale_match.location.match_size > 16) {
            ESP_LOGI(TAG, "fr %u t %ums thresh %u (x %d y %d r %f m %d) row %d/%d col %d/%d \n",
                    frame_count, (uint32_t)((end_time - start_time) / 1000),
                    (loc_ctx.threshold[0] + loc_ctx.threshold[1]) / 2,
                    loc_ctx.scale_match.location.x, loc_ctx.scale_match.location.y,
                    180 *
                            atan2(loc_ctx.scale_match.location.rotation.y,
                                    loc_ctx.scale_match.location.rotation.x) /
                            M_PI,
                    loc_ctx.scale_match.location.match_size, loc_ctx.scale_match.row_code.n_errors,
                    loc_ctx.scale_match.row_code.n_samples, loc_ctx.scale_match.col_code.n_errors,
                    loc_ctx.scale_match.col_code.n_samples);
        }
        start_time = end_time;
    }
}

void app_main() {
    app_wifi_main();
    app_camera_main();
    app_httpd_main();
    xTaskCreatePinnedToCore(main_loop, "main_loop", 8192, NULL, 9, NULL, 1);
}

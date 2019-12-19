#include "esp_timer.h"
#include "esp_camera.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "tests.h"
#include "location_decode.h"
#include "mls_query.h"

/* externals */
void app_camera_main();
void app_wifi_main();
void app_httpd_main();

extern QueueHandle_t frame_queues[];

/* static data */
static const char* TAG = "main_loop";

static uint8_t buf_64x64[6][64 * 64];
// static uint8_t buf_32x32[4][32 * 32];

static camera_fb_t double_buffers[][2] = {
        {
                {buf_64x64[0], 62 * 62, 62, 62, PIXFORMAT_GRAYSCALE},
                {buf_64x64[1], 62 * 62, 62, 62, PIXFORMAT_GRAYSCALE},
        },
        {
                {buf_64x64[2], 62 * 62, 62, 62, PIXFORMAT_GRAYSCALE},
                {buf_64x64[3], 62 * 62, 62, 62, PIXFORMAT_GRAYSCALE},
        },
        {
                {buf_64x64[4], 62 * 62, 62, 62, PIXFORMAT_GRAYSCALE},
                {buf_64x64[5], 62 * 62, 62, 62, PIXFORMAT_GRAYSCALE},
        },
#if 0
        {
                {buf_32x32[0], 32 * 32, 32, 32, PIXFORMAT_GRAYSCALE},
                {buf_32x32[1], 32 * 32, 32, 32, PIXFORMAT_GRAYSCALE},
        },
        {
                {buf_32x32[2], 32 * 32, 32, 32, PIXFORMAT_GRAYSCALE},
                {buf_32x32[3], 32 * 32, 32, 32, PIXFORMAT_GRAYSCALE},
        },
#endif
};

#define N_DOUBLE_BUFFERS (sizeof(double_buffers) / sizeof(double_buffers[0]))

static camera_fb_t* claimed_buffers[N_DOUBLE_BUFFERS + 1] = {};

/* helper functions */

static inline ImageMatrix fb_to_img(camera_fb_t fb) {
    return (ImageMatrix){fb.buf, fb.width, fb.height};
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
    assert(queue_index < N_DOUBLE_BUFFERS + 1);
    assert(!claimed_buffers[queue_index]);
    xQueueReceive(frame_queues[queue_index], &claimed_buffers[queue_index], 0);
    assert(claimed_buffers[queue_index]);
    if (queue_index == 0) {
        claimed_buffers[0] = camera_fb_swap(claimed_buffers[0]);
    }
    return fb_to_img(*claimed_buffers[queue_index]);
}

static void queue_fb_return(uint8_t queue_index) {
    assert(queue_index < N_DOUBLE_BUFFERS + 1);
    assert(claimed_buffers[queue_index]);
    if (pdTRUE == xQueueSendToBack(frame_queues[queue_index], &claimed_buffers[queue_index], 0)) {
        claimed_buffers[queue_index] = NULL;
    }
    assert(!claimed_buffers[queue_index]);
}

/* run */
static void main_loop(void* pvParameters) {
    // initialize queues
    for (int i = 0; i <= sizeof(double_buffers) / sizeof(double_buffers[0]); ++i) {
        for (int j = 0; j < 2; ++j) {
            camera_fb_t* fb_ptr = i ? &double_buffers[i - 1][j] : esp_camera_fb_get();
            xQueueSendToBack(frame_queues[i], &fb_ptr, 0);
        }
        assert(!uxQueueSpacesAvailable(frame_queues[i]));
    }
    int64_t start_time = esp_timer_get_time();
    // main loop
    for (;;) {
        ImageMatrix original_image = queue_fb_get(0);
        img_normalize(&original_image, original_image);
        Vector2f rotation = img_estimate_rotation(original_image);

        ImageMatrix thresholded_image = queue_fb_get(1);
        img_edge_hysteresis_threshold(&thresholded_image, original_image, 0, 100);
        queue_fb_return(1);

        ImageMatrix unrotated_image = queue_fb_get(2);
        // rotation.x *= 0.5f;
        // rotation.y *= -0.5f;
        rotation.y *= -1.0f;
        // img_rotate(unrotated_image, original_image, rotation, pixel_average);
        // img_rotate(unrotated_image, original_image, rotation, pixel_average);
        img_edge_hysteresis_threshold(&unrotated_image, original_image, 40, 255);

        ImageMatrix final_image = queue_fb_get(3);
        img_edge_hysteresis_threshold(&final_image, original_image, 30, 80);
        queue_fb_return(0);
        queue_fb_return(2);
        queue_fb_return(3);

        // end loop
        int64_t end_time = esp_timer_get_time();
        ESP_LOGI(TAG, "%ums %f", (uint32_t)((end_time - start_time) / 1000),
                180 * atan2(rotation.y, rotation.x) / M_PI);
        start_time = end_time;
    }
}

void app_main() {
    // assert(!run_all_tests());
    // ESP_LOGI(TAG, "%s\ntests ran: %d\n", test_error, test_count);

    app_wifi_main();
    app_camera_main();
    app_httpd_main();
    xTaskCreatePinnedToCore(main_loop, "main_loop", 5000, NULL, 9, NULL, 1);
}

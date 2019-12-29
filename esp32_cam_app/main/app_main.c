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

static const uint8_t double_buffer_sizes[] = {64, 62, 62};
static camera_fb_t double_buffers[sizeof(double_buffer_sizes)][2];
static camera_fb_t* claimed_buffers[sizeof(double_buffer_sizes) + 1] = {};

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
    assert(queue_index < sizeof(double_buffer_sizes) + 1);
    assert(!claimed_buffers[queue_index]);
    xQueueReceive(frame_queues[queue_index], &claimed_buffers[queue_index], 0);
    assert(claimed_buffers[queue_index]);
    if (queue_index == 0) {
        claimed_buffers[0] = camera_fb_swap(claimed_buffers[0]);
    }
    return fb_to_img(*claimed_buffers[queue_index]);
}

static void queue_fb_return(uint8_t queue_index) {
    assert(queue_index < sizeof(double_buffer_sizes) + 1);
    assert(claimed_buffers[queue_index]);
    if (pdTRUE == xQueueSendToBack(frame_queues[queue_index], &claimed_buffers[queue_index], 0)) {
        claimed_buffers[queue_index] = NULL;
    }
    assert(!claimed_buffers[queue_index]);
}

/* run */
static void main_loop(void* pvParameters) {
    // allocate double buffers
    for (int i = 0; i <= sizeof(double_buffer_sizes); ++i) {
        for (int j = 0; j < 2; ++j) {
            int16_t n = double_buffer_sizes[i];
            double_buffers[i][j] =
                    (camera_fb_t){heap_caps_malloc(n * n, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL),
                            n * n, n, n, PIXFORMAT_GRAYSCALE};
            assert(double_buffers[i][j].buf);
        }
    }
    // initialize queues
    for (int i = 0; i <= sizeof(double_buffer_sizes); ++i) {
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
        ImageMatrix unrotated_image = queue_fb_get(1);
        ImageMatrix thresholded_image = queue_fb_get(2);
        ImageMatrix final_image = queue_fb_get(3);

        IMG_NORMALIZE(&original_image, original_image);
        int32_t pixel_average = 0;
        IMG_SUM(pixel_average, original_image);
        pixel_average /= IMG_SIZE(original_image);
        Vector2f rotation = img_estimate_rotation(original_image);
        rotation.y *= -1.0f;
        img_rotate(unrotated_image, original_image, rotation, pixel_average,
                img_bilinear_interpolation);
        rotation = v2f_scale(rotation, 0.5f);
        img_rotate(thresholded_image, original_image, rotation, pixel_average,
                img_bilinear_interpolation);
        img_edge_hysteresis_threshold(&final_image, unrotated_image, 30, pixel_average, 60);

        queue_fb_return(0);
        queue_fb_return(1);
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

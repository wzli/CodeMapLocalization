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

static uint8_t buf_64x64[2][64 * 64];
static uint8_t buf_32x32[4][32 * 32];

static camera_fb_t double_buffers[][2] = {
        {
                {buf_64x64[0], 64 * 64, 64, 64, PIXFORMAT_GRAYSCALE},
                {buf_64x64[1], 64 * 64, 64, 64, PIXFORMAT_GRAYSCALE},
        },
        {
                {buf_32x32[0], 32 * 32, 32, 32, PIXFORMAT_GRAYSCALE},
                {buf_32x32[1], 32 * 32, 32, 32, PIXFORMAT_GRAYSCALE},
        },
        {
                {buf_32x32[2], 32 * 32, 32, 32, PIXFORMAT_GRAYSCALE},
                {buf_32x32[3], 32 * 32, 32, 32, PIXFORMAT_GRAYSCALE},
        },
};

/* helper functions */
static inline ImageMatrix fb_to_img(camera_fb_t fb) {
    return (ImageMatrix){fb.buf, fb.width, fb.height};
}

static inline camera_fb_t* queue_fb_get(QueueHandle_t frame_queue) {
    camera_fb_t* fb_ptr = NULL;
    xQueueReceive(frame_queue, &fb_ptr, 0);
    assert(fb_ptr);
    return fb_ptr;
}

static inline int queue_fb_return(QueueHandle_t frame_queue, const camera_fb_t* fb_ptr) {
    int success = xQueueSendToBack(frame_queue, &fb_ptr, 0);
    assert(success == pdTRUE);
    return success;
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
        // get 0
        esp_camera_fb_return(queue_fb_get(frame_queues[0]));
        camera_fb_t* fb_ptr_0 = esp_camera_fb_get();
        while (!fb_ptr_0) {
            ESP_LOGE(TAG, "Camera capture failed");
            fb_ptr_0 = esp_camera_fb_get();
        }

        // process 0
        ImageMatrix img = fb_to_img(*fb_ptr_0);
        Vector2f rot = img_estimate_rotation(img);

        // get 1
        camera_fb_t* fb_ptr_1 = queue_fb_get(frame_queues[1]);

        // process 0 -> 1
        ImageMatrix copy_img = fb_to_img(*fb_ptr_1);
        img_normalize(&copy_img, img);

        // return 0
        queue_fb_return(frame_queues[0], fb_ptr_0);
        // return 1
        queue_fb_return(frame_queues[1], fb_ptr_1);

        // end loop
        int64_t end_time = esp_timer_get_time();
        ESP_LOGI(TAG, "%ums %f", (uint32_t)((end_time - start_time) / 1000),
                180 * atan2(rot.y, rot.x) / M_PI);
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

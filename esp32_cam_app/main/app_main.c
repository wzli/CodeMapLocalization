#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "tests.h"
#include "location_decode.h"
#include "mls_query.h"

extern QueueHandle_t frame_queues[2];

typedef enum {
    ORIGINAL = 0,
    GRAYSCALE,
} StreamIndex;

void app_camera_main();
void app_httpd_main();
void app_wifi_main();

#define N_PIXELS (160 * 120)
static const char* TAG = "main_loop";
static uint8_t rgb_data[N_PIXELS * 3];
static uint8_t gray_idx = 0;
static uint8_t gray_data[2][N_PIXELS];
static camera_fb_t gray_fb[2];

static inline camera_fb_t img_to_fb(const ImageMatrix img) {
    return (camera_fb_t){
            img.data, img.n_cols * img.n_rows, img.n_cols, img.n_rows, PIXFORMAT_GRAYSCALE};
}

void main_loop(void* pvParameters) {
    camera_fb_t* stale_fb = NULL;
    xQueueSendToFront(frame_queues[GRAYSCALE], &stale_fb, 0);
    for (;;) {
        int64_t fr_start = esp_timer_get_time();
        camera_fb_t* fb = esp_camera_fb_get();

        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }

        if (!fmt2rgb888(fb->buf, fb->len, fb->format, rgb_data)) {
            esp_camera_fb_return(fb);
            ESP_LOGE(TAG, "to rgb888 failed, fb_len %uB", fb->len);
            continue;
        }

        ImageMatrix rgb_img = {rgb_data, fb->width, fb->height};

        if (pdTRUE == xQueueReceive(frame_queues[ORIGINAL], &stale_fb, 0)) {
            esp_camera_fb_return(stale_fb);
        }
        xQueueSendToFront(frame_queues[ORIGINAL], &fb, 0);

        ImageMatrix gray_img = {gray_data[gray_idx], 0, 0};
        img_convert_from_rgb888(&gray_img, rgb_img);

        if (pdTRUE == xQueueReceive(frame_queues[GRAYSCALE], &stale_fb, 0)) {
            fb = &gray_fb[gray_idx];
            *fb = img_to_fb(gray_img);
            xQueueSendToFront(frame_queues[GRAYSCALE], &fb, 0);
            gray_idx ^= 1;
        }

        Vector2f rot = img_estimate_rotation(gray_img);

        int64_t fr_end = esp_timer_get_time();
        ESP_LOGI(TAG, "%ums %f", (uint32_t)((fr_end - fr_start) / 1000),
                180 * atan2(rot.y, rot.x) / M_PI);
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

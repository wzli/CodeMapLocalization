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
void app_httpd_main();
void app_wifi_main();

extern QueueHandle_t frame_queues[];

/* typedefs */
typedef struct {
    QueueHandle_t* frame_queue;
    uint8_t* data[2];
    camera_fb_t fb[2];
    uint8_t cur;
} DoubleBuffer;

/* static data */
static const char* TAG = "main_loop";
static camera_fb_t* stale_fb = NULL;
static uint8_t db0_data[2][64 * 64];
static DoubleBuffer db0 = {&frame_queues[1], {db0_data[0], db0_data[1]}, {}, 0};

/* helper functions */
static inline camera_fb_t img_to_fb(const ImageMatrix img) {
    return (camera_fb_t){
            img.data, img.n_cols * img.n_rows, img.n_cols, img.n_rows, PIXFORMAT_GRAYSCALE};
}

static inline ImageMatrix db_take_frame(const DoubleBuffer* db) {
    return (ImageMatrix){db->data[db->cur], 0, 0};
}

static inline void db_give_frame(DoubleBuffer* db, const ImageMatrix mat) {
    if (pdTRUE != xQueueReceive(*db->frame_queue, &stale_fb, 0)) {
        return;
    }
    camera_fb_t* fb = &db->fb[db->cur];
    *fb = img_to_fb(mat);
    xQueueSendToFront(*db->frame_queue, &fb, 0);
    db->cur ^= 1;
}

/* run */
void main_loop(void* pvParameters) {
    for (;;) {
        int64_t fr_start = esp_timer_get_time();
        camera_fb_t* fb = esp_camera_fb_get();

        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }

        // process frame
        ImageMatrix img = {fb->buf, fb->width, fb->height};
        Vector2f rot = img_estimate_rotation(img);
        ImageMatrix copy_img = db_take_frame(&db0);
        img_copy(&copy_img, img);

        // add original frame to queue
        if (pdTRUE == xQueueReceive(frame_queues[0], &stale_fb, 0)) {
            esp_camera_fb_return(stale_fb);
        }
        xQueueSendToFront(frame_queues[0], &fb, 0);
        db_give_frame(&db0, copy_img);
        // end loop
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

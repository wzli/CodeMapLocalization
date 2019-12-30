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
static camera_fb_t* claimed_buffers[N_DOUBLE_BUFFERS + 1] = {};

static uint32_t histogram[256];

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

        Vector2f rotation = img_estimate_rotation(images[0]);
        rotation.y *= -(float) M_SQRT1_2;
        rotation.x *= (float) M_SQRT1_2;
#define BG_FILL (0)
        images[1].n_cols = 64;
        images[1].n_rows = 64;
        img_rotate(images[1], images[0], rotation, BG_FILL, img_bilinear_interpolation);
        img_filter(&images[2], images[1], sharpen_kernel);

        images[3].n_cols = 32;
        images[3].n_rows = 32;
        img_resize(images[3], images[2], img_bilinear_interpolation);
        img_histogram(histogram, images[3]);
        histogram[BG_FILL] -= IMG_SIZE(images[3]) / 2;
        uint8_t otsu_threshold = img_otsu_histogram_threshold(histogram);
        IMG_THRESHOLD(&images[3], images[3], otsu_threshold);

        img_histogram(histogram, images[2]);
        histogram[BG_FILL] -= IMG_SIZE(images[2]) / 2;
        otsu_threshold = img_otsu_histogram_threshold(histogram);
        IMG_THRESHOLD(&images[2], images[2], otsu_threshold);

        images[1].n_cols = 32;
        images[1].n_rows = 32;
        img_resize(images[1], images[2], img_bilinear_interpolation);
        img_histogram(histogram, images[1]);
        histogram[BG_FILL] -= IMG_SIZE(images[1]) / 2;
        otsu_threshold = img_otsu_histogram_threshold(histogram);
        IMG_THRESHOLD(&images[1], images[1], otsu_threshold);

        for (uint8_t i = 0; i <= N_DOUBLE_BUFFERS; ++i) {
            assert(claimed_buffers[i]);
            claimed_buffers[i]->width = images[i].n_cols;
            claimed_buffers[i]->height = images[i].n_rows;
            claimed_buffers[i]->len = IMG_SIZE(images[i]);
            queue_fb_return(i);
        }

        // end loop
        int64_t end_time = esp_timer_get_time();

        if (!(frame_count & 0xF)) {
            ESP_LOGI(TAG, "frame %u time %ums rot %fdeg thresh %u", frame_count,
                    (uint32_t)((end_time - start_time) / 1000),
                    180 * atan2(rotation.y, rotation.x) / M_PI, otsu_threshold);
        }
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

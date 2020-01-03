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
static BitMatrix64 binary_image;
static BitMatrix64 binary_mask;

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

        // find threshold of original image
        img_histogram(histogram, images[0]);
        uint8_t original_threshold = img_compute_otsu_threshold(histogram);

        // find rotation of original image
        Vector2f rotation = img_estimate_rotation(images[0]);
        rotation.y *= -(float) M_SQRT1_2;
        rotation.x *= (float) M_SQRT1_2;

        // unrotate and sharpen
        images[1].n_cols = 64;
        images[1].n_rows = 64;
        img_rotate(images[1], images[0], rotation, original_threshold, img_bilinear_interpolation);
        img_convolution_filter(&images[1], images[1], img_sharpen_kernel);

        // find threshold of filtered image
        img_histogram(histogram, images[1]);
        histogram[original_threshold] = 0;
        uint8_t filtered_threshold = img_compute_otsu_threshold(histogram);
        if (filtered_threshold < original_threshold) {
            SWAP(filtered_threshold, original_threshold);
        }
        // binarize to bit matrix
        img_to_bm64(binary_image, binary_mask, images[1], original_threshold, filtered_threshold);
        bm64_to_img(&images[2], binary_image, binary_mask);

        // extract row and column codes
        AxisCode64 row_code, col_code;
        bm64_extract_axis_codes(&row_code, &col_code, binary_image, binary_mask, 5);

        // display results
        bm64_from_axis_codes(binary_image, binary_mask, row_code, col_code);
        bm64_to_img(&images[3], binary_image, binary_mask);

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
                    180 * atan2(rotation.y, rotation.x) / M_PI, original_threshold);
        }
        start_time = end_time;
    }
}

void app_main() {
    // assert(!run_all_tests());

    app_wifi_main();
    app_camera_main();
    app_httpd_main();
    xTaskCreatePinnedToCore(main_loop, "main_loop", 10000, NULL, 9, NULL, 1);
}

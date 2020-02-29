#include "sdkconfig.h"
#include "esp_camera.h"
#include "esp_dsp.h"
#include "esp_log.h"
#include "esp_timer.h"
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
static BitMatrix64 binary_image_64;
static BitMatrix64 binary_mask_64;
static BitMatrix32 binary_image_32;
static BitMatrix32 binary_mask_32;

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
        uint8_t threshold0 = img_compute_otsu_threshold(histogram);

        // find rotation of original image
        Vector2f rotation = img_estimate_rotation(images[0]);
        if (!v2f_is_zero(rotation)) {
            // unrotate
            rotation.y *= -1;
            IMG_SET_SIZE(images[1], 64, 64);
            img_rotate(images[1], images[0], rotation, threshold0, img_bilinear_interpolation);
        }
        // sharpen
        img_hyper_sharpen(&images[1], images[1]);
        Vector2f vertex = v2f_rotate(
                rotation, (Vector2f){2 + images[1].size.x / 2, 2 + images[1].size.y / 2});
        img_draw_regular_polygon(images[1],
                (ImagePoint){images[1].size.x / 2, images[1].size.y / 2}, vertex, 4, threshold0, 5);

        // find threshold of filtered image
        img_histogram(histogram, images[1]);
        histogram[threshold0] = 0;
        uint8_t threshold1 = img_compute_otsu_threshold(histogram);
        if (threshold1 < threshold0) {
            SWAP(threshold1, threshold0);
        }
        // binarize to bit matrix
        img_to_bm64(binary_image_64, binary_mask_64, images[1], threshold0, threshold1);
        bm64_to_img(&images[2], binary_image_64, binary_mask_64);

        // extract row and column codes
        ScaleQuery query = {{}, {}, 0.8f, 1.2f, 0.02f};
        bm64_extract_axiscodes(
                &query.row_code, &query.col_code, binary_image_64, binary_mask_64, 5);
        ScaleMatch match = {};
        scale_search_location(&match, &query);

        match.location.rotation = v2f_add_angle(match.location.rotation, rotation);

        // display results
        bm32_from_axiscodes(binary_image_32, binary_mask_32, match.row_code, match.col_code);
        bm32_to_img(&images[3], binary_image_32, binary_mask_32);

        for (uint8_t i = 0; i <= N_DOUBLE_BUFFERS; ++i) {
            assert(claimed_buffers[i]);
            claimed_buffers[i]->width = images[i].size.x;
            claimed_buffers[i]->height = images[i].size.y;
            claimed_buffers[i]->len = IMG_PIXEL_COUNT(images[i]);
            queue_fb_return(i);
        }

        // end loop
        int64_t end_time = esp_timer_get_time();
        if (match.location.match_size > 16) {
            ESP_LOGI(TAG, "fr %u t %ums thresh %u (x %d y %d r %f m %d) row %d/%d col %d/%d \n",
                    frame_count, (uint32_t)((end_time - start_time) / 1000),
                    (threshold0 + threshold1) / 2, match.location.x, match.location.y,
                    180 * atan2(match.location.rotation.y, match.location.rotation.x) / M_PI,
                    match.location.match_size, match.row_code.n_errors, match.row_code.n_samples,
                    match.col_code.n_errors, match.col_code.n_samples);
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

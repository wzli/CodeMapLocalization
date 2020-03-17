#include "sdkconfig.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

QueueHandle_t record_frame_queue;
uint32_t record_frame_count = 0;

static const char* TAG = "record";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\n\r\n";

static esp_err_t init_sdcard() {
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;                     // configure 1-line mode
    gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);  // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);   // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);  // D3, needed in 4- and 1-line modes
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false, .max_files = 5, .allocation_unit_size = 16 * 1024};
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s).", esp_err_to_name(ret));
        }
        return ret;
    }
    sdmmc_card_print_info(stdout, card);
    return ESP_OK;
}

static size_t jpg_encode_stream(void* arg, size_t index, const void* data, size_t len) {
    uint32_t bytes_written;
    FIL* rec = (FIL*) arg;
    FRESULT fresult = f_write(rec, data, len, &bytes_written);
    if (fresult != FR_OK) {
        ESP_LOGE(TAG, "failed to write jpeg chunk error %d", fresult);
    }
    return len;
}

static void record_loop(void* pvParameters) {
    char text_buf[16] = {};
    uint32_t path_index = 0;
    FRESULT fresult = FR_OK;
    // create new folder
    ESP_LOGI(TAG, "create new folder ...");
    do {
        sprintf(text_buf, "%d", path_index);
        fresult = f_mkdir(text_buf);
    } while (FR_OK != fresult && ++path_index);
    // create new recording
    sprintf(text_buf, "%d/mjpeg.rec", path_index);
    ESP_LOGI(TAG, "creating %s ...", text_buf);
    static FIL rec;
    fresult = f_open(&rec, text_buf, FA_WRITE | FA_CREATE_NEW);
    if (fresult != FR_OK) {
        ESP_LOGE(TAG, "failed to create %s error %d", text_buf, fresult);
    }
    // initialize mjepg
    const size_t STREAM_BOUNDARY_LEN = strlen(STREAM_BOUNDARY);
    const size_t STREAM_CONTENT_TYPE_LEN = strlen(STREAM_CONTENT_TYPE);
    const size_t STREAM_PART_LEN = strlen(STREAM_PART);
    uint32_t bytes_written = 0;
    f_write(&rec, STREAM_CONTENT_TYPE, STREAM_CONTENT_TYPE_LEN, &bytes_written);
    f_write(&rec, STREAM_BOUNDARY, STREAM_BOUNDARY_LEN, &bytes_written);
    // write frames
    for (; record_frame_count < CONFIG_RECORD_RAW_FRAMES_LIMIT; ++record_frame_count) {
        camera_fb_t* fb_ptr;
        xQueueReceive(record_frame_queue, &fb_ptr, portMAX_DELAY);
        f_write(&rec, STREAM_PART, STREAM_PART_LEN, &bytes_written);
        if (!frame2jpg_cb(fb_ptr, 80, jpg_encode_stream, &rec)) {
            ESP_LOGE(TAG, "failed to encode jpeg");
        }
        f_write(&rec, STREAM_BOUNDARY, STREAM_BOUNDARY_LEN, &bytes_written);
        if ((record_frame_count & 0x1FF) == 0) {
            fresult = f_sync(&rec);
            if (fresult != FR_OK) {
                ESP_LOGE(TAG, "failed to sync frame error %d", fresult);
            }
        }
    }
}

void app_record_main() {
    // init sd card
    if (ESP_OK != init_sdcard()) {
        return;
    }
#if CONFIG_RECORD_RAW_FRAMES_ENABLE
    // init record queue
    record_frame_queue = xQueueCreate(1, sizeof(camera_fb_t*));
    // create record task
    xTaskCreatePinnedToCore(record_loop, "record", 4096, NULL, 7, NULL, 0);
#endif
}

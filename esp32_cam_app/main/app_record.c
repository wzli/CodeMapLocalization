#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
//#undef MIN
//#undef MAX

static const char* TAG = "record";

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

void app_record_main() {
    // init sd card
    if (ESP_OK != init_sdcard()) {
        return;
    }
    char file_path[32] = {};
    int path_index = 0;
    FRESULT result = FR_OK;
    do {
        ++path_index;
        snprintf(file_path, 32, "%d", path_index++);
        result = f_mkdir(file_path);
    } while (FR_OK != result);
}

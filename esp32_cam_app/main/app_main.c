#include "esp_timer.h"
#include "esp_camera.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

extern QueueHandle_t raw_frame_queue;
void app_camera_main();
void app_httpd_main();
void app_wifi_main();

void main_loop(void* pvParameters) {
    for(;;) {
        camera_fb_t* fb = esp_camera_fb_get();

       // Do some stuff ...


        camera_fb_t* stale_fb = NULL;
        if(pdTRUE == xQueueReceive(raw_frame_queue, &stale_fb, 0)) {
            esp_camera_fb_return(stale_fb);
        }
        if(pdTRUE != xQueueSendToFront(raw_frame_queue, &fb, 0)) {
            esp_camera_fb_return(fb);
        }
    }
}

void app_main() {
    app_wifi_main();
    app_camera_main();
    app_httpd_main();
    xTaskCreatePinnedToCore(main_loop, "main_loop", 10000, NULL, 9, NULL, 1);
}

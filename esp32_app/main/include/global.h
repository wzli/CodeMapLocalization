#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#undef MAX
#undef MIN
#include "debug_prints.h"

#define N_FRAME_QUEUES 4

extern TaskHandle_t record_task;
extern TaskHandle_t csv_log_task;
extern QueueHandle_t frame_queues[];

extern const int* led_feedback;
extern uint32_t record_frame_count;
extern LocalizationContext loc_ctx;

void app_camera_main();
void app_wifi_main();
void app_httpd_main();
void app_record_main();

void set_flash_led(int level);

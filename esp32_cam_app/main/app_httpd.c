#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/queue.h"

/* global variables */

#define N_FRAME_QUEUES 4
QueueHandle_t frame_queues[N_FRAME_QUEUES] = {};

static const char* TAG = "web_ui";
static char text_buf[1024];
static int led_duty = -1;

/* helper functions */

typedef struct {
    httpd_req_t* req;
    size_t len;
} jpg_chunking_t;

static size_t jpg_encode_stream(void* arg, size_t index, const void* data, size_t len) {
    jpg_chunking_t* j = (jpg_chunking_t*) arg;
    if (!index) {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char*) data, len) != ESP_OK) {
        return 0;
    }
    j->len += len;
    return len;
}

void set_led(int duty) {
#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
#ifdef CONFIG_LED_LEDC_LOW_SPEED_MODE
#define CONFIG_LED_LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#else
#define CONFIG_LED_LEDC_SPEED_MODE LEDC_HIGH_SPEED_MODE
#endif
    if (duty > CONFIG_LED_MAX_INTENSITY) {
        duty = CONFIG_LED_MAX_INTENSITY;
    }
    ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
    ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
    ESP_LOGI(TAG, "Set LED intensity to %d", duty);
    led_duty = duty;
#endif
}

static int print_heap_info(char* buf, const char* name, uint32_t capabilities) {
    multi_heap_info_t heap_info;
    heap_caps_get_info(&heap_info, capabilities);
    return sprintf(buf,
            "\n%s Heap Info\n"
            "total free bytes \t\t%d\n"
            "total allocated bytes \t\t%d\n"
            "largest free block \t\t%d\n"
            "minimum free bytes \t\t%d\n"
            "allocated blocks \t\t%d\n"
            "free blocks \t\t\t%d\n"
            "total blocks \t\t\t%d\n",
            name, heap_info.total_free_bytes, heap_info.total_allocated_bytes,
            heap_info.largest_free_block, heap_info.minimum_free_bytes, heap_info.allocated_blocks,
            heap_info.free_blocks, heap_info.total_blocks);
}

static esp_err_t parse_request_param(httpd_req_t* req, const char* param_name, int* param) {
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len < 2 || (httpd_req_get_url_query_str(req, text_buf, buf_len) != ESP_OK) ||
            (httpd_query_key_value(text_buf, param_name, text_buf, 32) != ESP_OK)) {
        return ESP_FAIL;
    }
    *param = atoi(text_buf);
    return ESP_OK;
}

/* URI handlers */

static esp_err_t index_handler(httpd_req_t* req) {
    extern const unsigned char index_html_gz_start[] asm("_binary_index_html_gz_start");
    extern const unsigned char index_html_gz_end[] asm("_binary_index_html_gz_end");
    size_t index_html_gz_len = index_html_gz_end - index_html_gz_start;

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    sensor_t* s = esp_camera_sensor_get();
    if (s != NULL) {
        return httpd_resp_send(req, (const char*) index_html_gz_start, index_html_gz_len);
    } else {
        ESP_LOGE(TAG, "Camera sensor not found");
        return httpd_resp_send_500(req);
    }
}

static esp_err_t stats_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/plain");
#if defined(CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS) && \
        defined(CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS)
    static const char TASK_LIST_HEADER[] =
            "Task Name       State   Pri     Stack   Num     CoreId\n";
    httpd_resp_send_chunk(req, TASK_LIST_HEADER, sizeof(TASK_LIST_HEADER) - 1);
    vTaskList(text_buf);
    httpd_resp_send_chunk(req, text_buf, strlen(text_buf));
    static const char TASK_RUNTIME_STATS_HEADER[] = "\nTask Name       Abs Time        % Time\n";
    httpd_resp_send_chunk(req, TASK_RUNTIME_STATS_HEADER, sizeof(TASK_RUNTIME_STATS_HEADER) - 1);
    vTaskGetRunTimeStats(text_buf);
    httpd_resp_send_chunk(req, text_buf, strlen(text_buf));
#endif
    int stat_len = print_heap_info(text_buf, "Internal", MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    httpd_resp_send_chunk(req, text_buf, stat_len);
    stat_len = print_heap_info(text_buf, "SPIRAM", MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    httpd_resp_send_chunk(req, text_buf, stat_len);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t capture_handler(httpd_req_t* req) {
    int lossless = 0;
    parse_request_param(req, "lossless", &lossless);
    int stream;
    if (ESP_OK != parse_request_param(req, "stream", &stream) || stream < 0 ||
            stream >= N_FRAME_QUEUES) {
        stream = 0;
    }
    int64_t fr_start = esp_timer_get_time();
    camera_fb_t* fb = NULL;
    if (pdTRUE != xQueueReceive(frame_queues[stream], &fb, 20) && fb) {
        ESP_LOGE(TAG, "Frame Queue %d Timeout", stream);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    size_t fb_len = fb->len;
    assert(fb->format != PIXFORMAT_JPEG);
    esp_err_t result = ESP_OK;
    result |= httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if(lossless) {
        httpd_resp_set_type(req, "image/x-portable-graymap");
        httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.pgm");
        sprintf(text_buf, "P5\n%u %u\n%u\n", fb->width, fb->height, 255);
        result |= httpd_resp_send_chunk(req, text_buf, strlen(text_buf));
        result |= httpd_resp_send_chunk(req, (char*)fb->buf, fb->width * fb->height);
    }
    else {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
        jpg_chunking_t jchunk = {req, 0};
        result |= frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
        fb_len = jchunk.len;
    }
    result |= httpd_resp_send_chunk(req, NULL, 0);
    while (pdTRUE != xQueueSendToBack(frame_queues[stream], &fb, 5)) {
        ESP_LOGE(TAG, "Frame Queue %d Overflow", stream);
    }
    int64_t fr_end = esp_timer_get_time();
    ESP_LOGD(TAG, "JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    return result;
}

static esp_err_t cam_params_handler(httpd_req_t* req) {
    sensor_t* s = esp_camera_sensor_get();
    char* p = text_buf;
    *p++ = '{';
    p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p += sprintf(p, "\"awb\":%u,", s->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p += sprintf(p, "\"aec\":%u,", s->status.aec);
    p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p += sprintf(p, "\"agc\":%u,", s->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p += sprintf(p, "\"vflip\":%u,", s->status.vflip);
    p += sprintf(p, "\"colorbar\":%u,", s->status.colorbar);
    p += sprintf(p, "\"led_intensity\":%d", led_duty);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, text_buf, strlen(text_buf));
}

static esp_err_t set_led_intensity_handler(httpd_req_t* req) {
    int val;
    if (ESP_OK != parse_request_param(req, "val", &val)) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    set_led(val);
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t set_cam_param_handler(httpd_req_t* req) {
    ESP_LOGI(TAG, "received cam param request: %s", req->uri);
    int val;
    if (ESP_OK != parse_request_param(req, "val", &val)) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    sensor_t* s = esp_camera_sensor_get();
    if ((*(int (*)(sensor_t*, int)) req->user_ctx)(s, val)) {
        return httpd_resp_send_500(req);
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

/* http server init */

void app_httpd_main() {
    // create queues
    for (int i = 0; i < N_FRAME_QUEUES; ++i) {
        frame_queues[i] = xQueueCreate(2, sizeof(camera_fb_t*));
        assert(frame_queues[i]);
    }
    sensor_t* s = esp_camera_sensor_get();
    httpd_uri_t uris[] = {
            {"/", HTTP_GET, index_handler, NULL},
            {"/stats", HTTP_GET, stats_handler, NULL},
            {"/capture", HTTP_GET, capture_handler, NULL},
            {"/set_led_intensity", HTTP_GET, set_led_intensity_handler, NULL},
            {"/cam_params", HTTP_GET, cam_params_handler, NULL},
            // uris for setting camera params
            {"/set_brightness", HTTP_GET, set_cam_param_handler, s->set_brightness},
            {"/set_contrast", HTTP_GET, set_cam_param_handler, s->set_contrast},
            {"/set_saturation", HTTP_GET, set_cam_param_handler, s->set_saturation},
            {"/set_special_effect", HTTP_GET, set_cam_param_handler, s->set_special_effect},
            {"/set_awb", HTTP_GET, set_cam_param_handler, s->set_whitebal},
            {"/set_awb_gain", HTTP_GET, set_cam_param_handler, s->set_awb_gain},
            {"/set_wb_mode", HTTP_GET, set_cam_param_handler, s->set_wb_mode},
            {"/set_aec", HTTP_GET, set_cam_param_handler, s->set_exposure_ctrl},
            {"/set_aec2", HTTP_GET, set_cam_param_handler, s->set_aec2},
            {"/set_aec_value", HTTP_GET, set_cam_param_handler, s->set_aec_value},
            {"/set_ae_level", HTTP_GET, set_cam_param_handler, s->set_ae_level},
            {"/set_agc", HTTP_GET, set_cam_param_handler, s->set_gain_ctrl},
            {"/set_agc_gain", HTTP_GET, set_cam_param_handler, s->set_agc_gain},
            {"/set_gainceiling", HTTP_GET, set_cam_param_handler, s->set_gainceiling},
            {"/set_raw_gma", HTTP_GET, set_cam_param_handler, s->set_raw_gma},
            {"/set_lenc", HTTP_GET, set_cam_param_handler, s->set_lenc},
            {"/set_bpc", HTTP_GET, set_cam_param_handler, s->set_bpc},
            {"/set_wpc", HTTP_GET, set_cam_param_handler, s->set_wpc},
            {"/set_hmirror", HTTP_GET, set_cam_param_handler, s->set_hmirror},
            {"/set_vflip", HTTP_GET, set_cam_param_handler, s->set_vflip},
            {"/set_colorbar", HTTP_GET, set_cam_param_handler, s->set_colorbar},
    };
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 0;
    config.max_uri_handlers = sizeof(uris) / sizeof(httpd_uri_t);
    httpd_handle_t httpd;
    if (httpd_start(&httpd, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Starting web server on port: '%d'", config.server_port);
        for (int i = 0; i < config.max_uri_handlers; ++i) {
            int err = httpd_register_uri_handler(httpd, uris + i);
            ESP_LOGI(TAG, "registered URI handler at %s err %d\n", uris[i].uri, err);
        }
    }
    set_led(0);
}

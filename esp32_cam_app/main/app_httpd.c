#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "esp_log.h"

static const char* TAG = "web_ui";

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
int led_duty = 0;
bool isStreaming = false;

#ifdef CONFIG_LED_LEDC_LOW_SPEED_MODE
#define CONFIG_LED_LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#else
#define CONFIG_LED_LEDC_SPEED_MODE LEDC_HIGH_SPEED_MODE
#endif
#endif

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
void enable_led(bool en) {  // Turn LED On or Off
    int duty = en ? led_duty : 0;
    if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY)) {
        duty = CONFIG_LED_MAX_INTENSITY;
    }
    ledc_set_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
    ledc_update_duty(CONFIG_LED_LEDC_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
    ESP_LOGI(TAG, "Set LED intensity to %d", duty);
}
#endif

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

static esp_err_t capture_handler(httpd_req_t* req) {
    camera_fb_t* fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    enable_led(true);
    vTaskDelay(150 / portTICK_PERIOD_MS);  // The LED needs to be turned on ~150ms before the call
                                           // to esp_camera_fb_get()
    fb = esp_camera_fb_get();  // or it won't be visible in the frame. A better way to do this is
                               // needed.
    enable_led(false);
#else
    fb = esp_camera_fb_get();
#endif

    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG) {
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char*) fb->buf, fb->len);
    } else {
        jpg_chunking_t jchunk = {req, 0};
        res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
        httpd_resp_send_chunk(req, NULL, 0);
        fb_len = jchunk.len;
    }
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    ESP_LOGI(TAG, "JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    return res;
}

static esp_err_t stream_handler(httpd_req_t* req) {
    camera_fb_t* fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t* _jpg_buf = NULL;
    char* part_buf[64];

    static int64_t last_frame = 0;
    if (!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    enable_led(true);
    isStreaming = true;
#endif

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
        } else {
            if (fb->format != PIXFORMAT_JPEG) {
                bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                esp_camera_fb_return(fb);
                fb = NULL;
                if (!jpeg_converted) {
                    ESP_LOGE(TAG, "JPEG compression failed");
                    res = ESP_FAIL;
                }
            } else {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }
        if (res == ESP_OK) {
            size_t hlen = snprintf((char*) part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char*) part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char*) _jpg_buf, _jpg_buf_len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (fb) {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if (_jpg_buf) {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK) {
            break;
        }
        int64_t fr_end = esp_timer_get_time();

        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        ESP_LOGI(TAG, "MJPG: %uB %ums (%.1ffps)", (uint32_t)(_jpg_buf_len), (uint32_t) frame_time,
                1000.0 / (uint32_t) frame_time);
    }

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    isStreaming = false;
    enable_led(false);
#endif

    last_frame = 0;
    return res;
}

static esp_err_t cmd_handler(httpd_req_t* req) {
    char* buf;
    size_t buf_len;
    char variable[32] = {
            0,
    };
    char value[32] = {
            0,
    };

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        // TODO: change to fixed sized bffer
        buf = (char*) malloc(buf_len);
        if (!buf) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                    httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    ESP_LOGI(TAG, "%s = %d", variable, val);
    sensor_t* s = esp_camera_sensor_get();
    int res = 0;

    if (!strcmp(variable, "framesize")) {
        if (s->pixformat == PIXFORMAT_JPEG)
            res = s->set_framesize(s, (framesize_t) val);
    } else if (!strcmp(variable, "quality"))
        res = s->set_quality(s, val);
    else if (!strcmp(variable, "contrast"))
        res = s->set_contrast(s, val);
    else if (!strcmp(variable, "brightness"))
        res = s->set_brightness(s, val);
    else if (!strcmp(variable, "saturation"))
        res = s->set_saturation(s, val);
    else if (!strcmp(variable, "gainceiling"))
        res = s->set_gainceiling(s, (gainceiling_t) val);
    else if (!strcmp(variable, "colorbar"))
        res = s->set_colorbar(s, val);
    else if (!strcmp(variable, "awb"))
        res = s->set_whitebal(s, val);
    else if (!strcmp(variable, "agc"))
        res = s->set_gain_ctrl(s, val);
    else if (!strcmp(variable, "aec"))
        res = s->set_exposure_ctrl(s, val);
    else if (!strcmp(variable, "hmirror"))
        res = s->set_hmirror(s, val);
    else if (!strcmp(variable, "vflip"))
        res = s->set_vflip(s, val);
    else if (!strcmp(variable, "awb_gain"))
        res = s->set_awb_gain(s, val);
    else if (!strcmp(variable, "agc_gain"))
        res = s->set_agc_gain(s, val);
    else if (!strcmp(variable, "aec_value"))
        res = s->set_aec_value(s, val);
    else if (!strcmp(variable, "aec2"))
        res = s->set_aec2(s, val);
    else if (!strcmp(variable, "dcw"))
        res = s->set_dcw(s, val);
    else if (!strcmp(variable, "bpc"))
        res = s->set_bpc(s, val);
    else if (!strcmp(variable, "wpc"))
        res = s->set_wpc(s, val);
    else if (!strcmp(variable, "raw_gma"))
        res = s->set_raw_gma(s, val);
    else if (!strcmp(variable, "lenc"))
        res = s->set_lenc(s, val);
    else if (!strcmp(variable, "special_effect"))
        res = s->set_special_effect(s, val);
    else if (!strcmp(variable, "wb_mode"))
        res = s->set_wb_mode(s, val);
    else if (!strcmp(variable, "ae_level"))
        res = s->set_ae_level(s, val);
#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    else if (!strcmp(variable, "led_intensity")) {
        led_duty = val;
        if (isStreaming)
            enable_led(true);
    }
#endif
    else {
        res = -1;
    }

    if (res) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t* req) {
    static char json_response[1024];

    sensor_t* s = esp_camera_sensor_get();
    char* p = json_response;
    *p++ = '{';

    p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p += sprintf(p, "\"quality\":%u,", s->status.quality);
    p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p += sprintf(p, "\"awb\":%u,", s->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p += sprintf(p, "\"aec\":%u,", s->status.aec);
    p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p += sprintf(p, "\"agc\":%u,", s->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    p += sprintf(p, ",\"led_intensity\":%u", led_duty);
#else
    p += sprintf(p, ",\"led_intensity\":%d", -1);
#endif
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

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

void app_httpd_main() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t index_uri = {
            .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL};

    httpd_uri_t status_uri = {
            .uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL};

    httpd_uri_t cmd_uri = {
            .uri = "/control", .method = HTTP_GET, .handler = cmd_handler, .user_ctx = NULL};

    httpd_uri_t capture_uri = {
            .uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL};

    httpd_uri_t stream_uri = {
            .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL};

    ESP_LOGI(TAG, "Starting web server on port: '%d'", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    ESP_LOGI(TAG, "Starting stream server on port: '%d'", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}

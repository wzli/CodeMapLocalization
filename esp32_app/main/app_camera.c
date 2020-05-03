/* ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_camera.h"
#include "camera_pins.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const int* led_feedback;
static const char* TAG = "camera";

#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
void set_led_duty(int duty) {
    if (duty > CONFIG_LED_MAX_INTENSITY) {
        duty = CONFIG_LED_MAX_INTENSITY;
    }
#ifdef CONFIG_LED_LEDC_LOW_SPEED_MODE
    ledc_set_duty(LEDC_LOW_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
#else
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, CONFIG_LED_LEDC_CHANNEL);
#endif
}
#endif

#ifdef CONFIG_LED_AUTO_INTENSITY
int led_setpoint = CONFIG_LED_DEFAULT_INTENSITY;

static void led_control_loop(void* pvParameters) {
    int led_duty = CONFIG_LED_DEFAULT_INTENSITY;
    float feedback_average = led_duty;
    if (!led_feedback) {
        led_feedback = &led_duty;
    }
    while (true) {
        // run at 10 Hz
        vTaskDelay(10);
        // average feedback for 1s
        feedback_average *= 0.9f;
        feedback_average += *led_feedback * 0.1f;
        // control LED based on desired threshold
        if (led_duty < CONFIG_LED_MAX_INTENSITY && feedback_average < led_setpoint) {
            set_led_duty(++led_duty);
        } else if (led_duty > 0 && feedback_average > led_setpoint) {
            set_led_duty(--led_duty);
        }
    }
}
#endif

void set_flash_led(int level) {
    ESP_LOGI(TAG, "Set LED intensity to %d", level);
#ifdef CONFIG_LED_AUTO_INTENSITY
    led_setpoint = level;
#else
    set_led_duty(level);
#endif
}

static void flash_led_init() {
#ifdef CONFIG_LED_ILLUMINATOR_ENABLED
    gpio_set_direction(CONFIG_LED_LEDC_PIN, GPIO_MODE_OUTPUT);
    ledc_timer_config_t ledc_timer = {
            .duty_resolution = LEDC_TIMER_8_BIT,  // resolution of PWM duty
            .freq_hz = 1000,                      // frequency of PWM signal
            .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
            .timer_num = CONFIG_LED_LEDC_TIMER    // timer index
    };
    ledc_channel_config_t ledc_channel = {.channel = CONFIG_LED_LEDC_CHANNEL,
            .duty = 0,
            .gpio_num = CONFIG_LED_LEDC_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = CONFIG_LED_LEDC_TIMER};
#ifdef CONFIG_LED_LEDC_HIGH_SPEED_MODE
    ledc_timer.speed_mode = ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;
#endif
    switch (ledc_timer_config(&ledc_timer)) {
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(TAG, "ledc_timer_config() parameter error");
            break;
        case ESP_FAIL:
            ESP_LOGE(TAG,
                    "ledc_timer_config() Can not find a proper pre-divider number base on the "
                    "given frequency and the current duty_resolution");
            break;
        case ESP_OK:
            if (ledc_channel_config(&ledc_channel) == ESP_ERR_INVALID_ARG) {
                ESP_LOGE(TAG, "ledc_channel_config() parameter error");
            }
            break;
        default:
            break;
    }
    set_led_duty(CONFIG_LED_DEFAULT_INTENSITY);
#ifdef CONFIG_LED_AUTO_INTENSITY
    xTaskCreatePinnedToCore(led_control_loop, "led_control", 1024, NULL, 2, NULL, 0);
#endif
#endif
}

static void camera_init() {
#if CONFIG_CAMERA_MODEL_ESP_EYE
    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);
#endif

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    // init with high specs to pre-allocate larger buffers
    config.frame_size = FRAMESIZE_64x64;
    config.fb_count = 3;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t* s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);        // flip it back
        s->set_brightness(s, 1);   // up the blightness just a bit
        s->set_saturation(s, -2);  // lower the saturation
    }
    // drop down frame size for higher initial frame rate
    s->set_framesize(s, FRAMESIZE_64x64);
    // set contrast to highest (for better binary seperation)
    s->set_contrast(s, 2);

#ifndef CONFIG_CAMERA_AEC_ENABLE
    s->set_exposure_ctrl(s, 0);
    s->set_aec_value(s, CONFIG_CAMERA_DEFAULT_EXPOSURE);
#endif

#ifndef CONFIG_CAMERA_AGC_ENABLE
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, CONFIG_CAMERA_DEFAULT_GAIN);
#endif
}

void app_camera_main() {
    flash_led_init();
    camera_init();
}

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h" // *** Include LEDC driver for PWM ***
#include "driver/gpio.h" // *** Include GPIO driver for Status LED ***
#include "esp_log.h"

// ADC Settings (LDR on GPIO34 as per challenge diagram)
#define LDR_CHANNEL ADC1_CHANNEL_6  // GPIO34 is ADC1_CH6
#define DEFAULT_VREF    1100        // Use adc2_vref_to_gpio() for better estimate
#define NO_OF_SAMPLES   64          // Oversampling

// Buzzer Settings (Transistor Base on GPIO18, driven by PWM)
#define BUZZER_PIN      GPIO_NUM_18
#define BUZZER_THRESHOLD 1000       // ADC value below which the buzzer turns on

// LEDC (PWM) Settings for Passive Buzzer
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE // Use low speed mode
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT // Resolution 0-1023 (Duty cycle for 50% square wave)
#define LEDC_FREQUENCY  (2000) // Frequency in Hertz (ลองปรับค่านี้ได้ 1000-4000 Hz)

// *** Status LED Settings ***
#define LED_STATUS_PIN GPIO_NUM_16 // <<< เลือกขา GPIO สำหรับ LED ที่นี่

static const char *TAG = "LDR_PassiveBuzzer_LED"; // <<< เปลี่ยนชื่อ TAG เล็กน้อย
static esp_adc_cal_characteristics_t *adc_chars;
static bool is_buzzer_on = false; // Track buzzer state

static bool check_efuse(void)
{
    // Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Two Point: Supported");
    } else {
        ESP_LOGI(TAG, "eFuse Two Point: NOT supported");
    }
    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Vref: Supported");
    } else {
        ESP_LOGI(TAG, "eFuse Vref: NOT supported");
    }
    return true;
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "Characterized using Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "Characterized using eFuse Vref");
    } else {
        ESP_LOGI(TAG, "Characterized using Default Vref");
    }
}

// *** Function to turn buzzer ON using PWM ***
static void buzzer_on() {
    if (!is_buzzer_on) {
        // Set duty cycle to 50% to generate square wave
        uint32_t duty = (1 << (LEDC_DUTY_RES - 1)); // 512 for 10-bit resolution
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        is_buzzer_on = true;
        // ESP_LOGW(TAG, "Buzzer ON"); // Optional: อาจจะ Log เยอะไปถ้าทำงานถี่
    }
}

// *** Function to turn buzzer OFF using PWM ***
static void buzzer_off() {
    if (is_buzzer_on) {
        // Set duty cycle to 0%
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        is_buzzer_on = false;
        // ESP_LOGI(TAG, "Buzzer OFF"); // Optional: อาจจะ Log เยอะไปถ้าทำงานถี่
    }
}


void app_main(void)
{
    // Check eFuse characteristics
    check_efuse();

    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_CHANNEL, ADC_ATTEN_DB_11);

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    // *** Configure LEDC PWM Timer ***
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // *** Configure LEDC PWM Channel for Buzzer Pin ***
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BUZZER_PIN,
        .duty           = 0, // Start with duty 0 (off)
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    buzzer_off(); // Ensure buzzer is off initially

    // *** Configure Status LED GPIO pin as output ***
    gpio_reset_pin(LED_STATUS_PIN);
    gpio_set_direction(LED_STATUS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_STATUS_PIN, 1); // Turn Status LED ON permanently
    ESP_LOGI(TAG, "Status LED ON (GPIO%d)", LED_STATUS_PIN);

    ESP_LOGI(TAG, "LDR Passive Buzzer Alert with Status LED Initialized.");
    ESP_LOGI(TAG, "LDR on GPIO34, Buzzer PWM on GPIO18");
    ESP_LOGI(TAG, "Threshold: ADC < %d", BUZZER_THRESHOLD);

    while (1) {
        uint32_t adc_reading = 0;
        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)LDR_CHANNEL);
        }
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

        // Check if light level is below threshold
        if (adc_reading < BUZZER_THRESHOLD) {
            buzzer_on(); // *** Use buzzer_on() function ***
            ESP_LOGW(TAG, "ALERT! Low light detected. ADC: %d (%d mV)", adc_reading, voltage_mv);
        } else {
            buzzer_off(); // *** Use buzzer_off() function ***
            ESP_LOGI(TAG, "Light level OK. ADC: %d (%d mV)", adc_reading, voltage_mv);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Check more frequently
    }
}

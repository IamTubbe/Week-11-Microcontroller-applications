#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h" // Include LEDC driver for PWM
#include "esp_log.h"

// ADC Settings (LDR on GPIO34 as per challenge diagram)
#define LDR_CHANNEL ADC1_CHANNEL_6  // GPIO34 is ADC1_CH7
#define DEFAULT_VREF    1100        // Use adc2_vref_to_gpio() for better estimate
#define NO_OF_SAMPLES   64          // Oversampling

// LEDC (PWM) Settings for LED on GPIO18
#define LED_PIN         GPIO_NUM_18
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE // Use low speed mode for simplicity
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT // Resolution 0-1023
#define LEDC_FREQUENCY  (5000) // Frequency in Hertz

static const char *TAG = "LDR_LED_Control";
static esp_adc_cal_characteristics_t *adc_chars;

// Function to map ADC value (0-4095) to LEDC duty (0-1023)
static uint32_t map_adc_to_duty(uint32_t adc_value) {
    // Simple linear mapping
    // Ensure value stays within 0-4095 range first
    if (adc_value > 4095) adc_value = 4095;
    // Map to 0-1023 range (for 10-bit resolution)
    return (adc_value * 1023) / 4095;
}

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

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LED_PIN,
        .duty           = 0, // Set duty to 0% initially
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG, "LDR to LED Control Initialized.");
    ESP_LOGI(TAG, "LDR on GPIO34, LED on GPIO18");

    while (1) {
        uint32_t adc_reading = 0;
        // Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)LDR_CHANNEL);
        }
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

        // Map ADC reading to PWM duty cycle
        uint32_t duty_cycle = map_adc_to_duty(adc_reading);

        // Set LED brightness
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));

        ESP_LOGI(TAG, "Raw ADC: %d (%d mV), Mapped Duty: %d", adc_reading, voltage_mv, duty_cycle);

        vTaskDelay(pdMS_TO_TICKS(100)); // Delay 100ms
    }
}

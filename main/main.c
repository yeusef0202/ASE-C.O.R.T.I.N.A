#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_rom_sys.h"  // For esp_rom_delay_us

#define STEP_PIN    GPIO_NUM_3
#define DIR_PIN     GPIO_NUM_2
#define ENABLE_PIN  GPIO_NUM_7

#define STEPS_PER_REV    200    // Full-step count per revolution
#define MICROSTEP_FACTOR 1      // Microstep divisor (1 for full-step)
#define REVOLUTIONS      1      // Number of full rotations per cycle
#define STEP_DELAY_US    2000   // Delay between microsteps

static const char *TAG = "StepperApp";

void step_motor(int total_steps, bool direction) {
    gpio_set_level(DIR_PIN, direction);

    for (int i = 0; i < total_steps; i++) {
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_DELAY_US);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(STEP_DELAY_US);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing GPIOs");

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STEP_PIN) | (1ULL << DIR_PIN) | (1ULL << ENABLE_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(ENABLE_PIN, 0); // Enable driver (LOW = enabled)

    int steps_per_rotation = STEPS_PER_REV * MICROSTEP_FACTOR;
    int total_steps = steps_per_rotation * REVOLUTIONS;

    while (1) {
        ESP_LOGI(TAG, "Rotating %d full revolution(s) clockwise (%d steps)", REVOLUTIONS, total_steps);
        step_motor(total_steps, true);
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "Rotating %d full revolution(s) counter-clockwise (%d steps)", REVOLUTIONS, total_steps);
        step_motor(total_steps, false);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
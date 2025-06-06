#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_rom_sys.h"
// #include "esp_ble_gap.h"

#include "string.h"
#include <nvs_flash.h>

//---------------------------------------------

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_ota.h>

#include <esp_rmaker_common_events.h>

#include <app_network.h>
#include <app_insights.h>


#define STEP_PIN GPIO_NUM_3
#define DIR_PIN GPIO_NUM_2
#define ENABLE_PIN GPIO_NUM_6

#define SDA_PIN GPIO_NUM_7
#define SCL_PIN GPIO_NUM_9
#define I2C_PORT I2C_NUM_0
#define BH1750_ADDR 0x23

#define STEP_DELAY_US 1000
#define LIGHT_THRESHOLD_HALF 60.0f
#define HISTERESIS 5.0f
#define LIGHT_THRESHOLD 10.0f

#define UPPER_LIMIT 10000 // Define the full length of the Curtain
#define LOWER_LIMIT 0     // Fully open

static const char *TAG = "StepperApp";

static bool direction = true;
static int enable = 0;
static int step_count = 0; // Get the step position to determine when to stop ; 0- Full open, X - Full close
static int user_value = 0; // 0 - Auto, 1 - Down, 2 - Up, 3 - Stop
esp_rmaker_device_t *cortina_device;

static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
                                            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    // Get the user value from the parameter
    if(!strcmp(esp_rmaker_param_get_name(param), "Direction"))
    {
        const char *value = val.val.s;
        if (strcmp(value, "Up") == 0)
        {   
            user_value = 2;
        }
        else if (strcmp(value, "Down") == 0)
        {
            user_value = 1;
        }
        else if (strcmp(value, "Auto") == 0)
        {
            user_value = 0;
        }
        else if (strcmp(value, "Stop") == 0)
        {
            user_value = 3;
        }
        ESP_LOGI(TAG, "Direction set to: %s", value);
    }
    else
    {
        ESP_LOGW(TAG, "Unknown parameter: %s", esp_rmaker_param_get_name(param));
    }
    esp_rmaker_param_update(param, val);
    return ESP_OK;
}


void factory_reset() {
    nvs_flash_erase();
    esp_restart();
}

void stepper_task(void *arg)
{
    while (1)
    {
        gpio_set_level(ENABLE_PIN, enable);
        if (enable)
        {
            // ESP_LOGI(TAG, "Stepper motor is disabled");
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait before next check
            continue;
        }
        if (step_count > UPPER_LIMIT || step_count < LOWER_LIMIT)
        {
            step_count -= (direction ? 1 : -1);
            if (step_count >= UPPER_LIMIT)
                step_count = UPPER_LIMIT;
            else if (step_count <= LOWER_LIMIT)
                step_count = LOWER_LIMIT;
            ESP_LOGI(TAG, "Reached limit, stopping stepper motor");
            vTaskDelay(pdMS_TO_TICKS(1000)); // Wait before next check
            continue;
        }
        ESP_LOGI(TAG, "Current step count: %d", step_count);
        step_count -= (direction ? 1 : -1);

        gpio_set_level(DIR_PIN, direction);
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(STEP_DELAY_US);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(STEP_DELAY_US);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t bh1750_read_light(float *lux)
{
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x10, true); // Continuously H-Resolution Mode
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(180));

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK)
    {
        uint16_t raw = (data[0] << 8) | data[1];
        *lux = raw / 1.2f;
    }
    return ret;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing GPIOs");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STEP_PIN) | (1ULL << DIR_PIN) | (1ULL << ENABLE_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Initializing I2C");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000};
    ESP_LOGI(TAG, "I2C Config: SDA=%d, SCL=%d", SDA_PIN, SCL_PIN);
    i2c_param_config(I2C_PORT, &conf);
    ESP_LOGI(TAG, "Installing I2C driver");
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "I2C driver installed");
    
    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );


    // Start the ESP RainMaker core
    // esp_rmaker_console_init();

    // factory_reset();
    app_network_init();

    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "Cortina");
    if (!node)
    {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        abort();
    }

    cortina_device = esp_rmaker_device_create("Cortina", NULL, NULL);

    esp_rmaker_device_add_cb(cortina_device, write_cb, NULL);

    esp_rmaker_device_add_param(cortina_device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Cortina"));
    
    const char *dir_options[] = {"Up", "Down", "Auto", "Stop"};
    esp_rmaker_param_t *dir_param = esp_rmaker_param_create("Direction", NULL, esp_rmaker_str("Auto"), PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(dir_param, ESP_RMAKER_UI_DROPDOWN);
    esp_rmaker_param_add_valid_str_list(dir_param,dir_options, 4);
    esp_rmaker_device_add_param(cortina_device, dir_param);
    esp_rmaker_device_assign_primary_param(cortina_device, dir_param);
    

    esp_rmaker_node_add_device(node, cortina_device);

    esp_rmaker_start();

    app_wifi_start(POP_TYPE_RANDOM);


    xTaskCreate(stepper_task, "stepper_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Stepper task started");
    while (1)
    {

        switch (user_value)
        {
        case 0:
            float lux = 0.0;
            if (bh1750_read_light(&lux) == ESP_OK)
            {
                ESP_LOGI(TAG, "Light level: %.2f lux", lux);
                if (abs(LIGHT_THRESHOLD_HALF - lux + HISTERESIS) > LIGHT_THRESHOLD)
                {
                    ESP_LOGI(TAG, "Light level is above threshold, enabling stepper motor");
                    enable = 0;
                    direction = !(lux > LIGHT_THRESHOLD_HALF);
                }
                else
                {
                    // enable = 0;
                    ESP_LOGI(TAG, "Light level is below threshold, disabling stepper motor");
                    enable = 1; // Disable stepper motor
                }
            }
            else
            {
                ESP_LOGE(TAG, "Failed to read light sensor");
            }
            break;
        case 1:
            enable = 0;
            direction = 0;
            break;
        case 2:
            enable = 0;
            direction = 1;
            break;
        case 3:
            enable = 1; // Stop the stepper motor
            ESP_LOGI(TAG, "Stepper motor stopped by user command");
            break;
        default:
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
#include "reset_button.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "settings.h"

#define BUTTON_GPIO GPIO_NUM_14
#define BUTTON_PRESSED_LEVEL 0
#define POLL_INTERVAL_MS 50
#define HOLD_TIME_MS 5000
#define HOLD_POLLS (HOLD_TIME_MS / POLL_INTERVAL_MS)

static const char *TAG = "reset_button";

static void restore_dhcp(void)
{
    network_settings_t network;
    if (settings_load_network(&network) != ESP_OK) {
        ESP_LOGE(TAG, "could not read the network settings");
        return;
    }
    if (!network.eth_static) {
        ESP_LOGI(TAG, "ethernet already uses DHCP, nothing to do");
        return;
    }
    network.eth_static = false;
    if (settings_save_network(&network) != ESP_OK) {
        ESP_LOGE(TAG, "could not store the network settings");
        return;
    }
    ESP_LOGW(TAG, "ethernet reset to DHCP, restarting");
    esp_restart();
}

static void button_task(void *arg)
{
    int held_polls = 0;
    while (true) {
        if (gpio_get_level(BUTTON_GPIO) == BUTTON_PRESSED_LEVEL) {
            if (++held_polls == HOLD_POLLS) {
                restore_dhcp();
            }
        } else {
            held_polls = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

esp_err_t reset_button_start(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio");
    ESP_RETURN_ON_FALSE(xTaskCreate(button_task, "reset_button", 3072, NULL, 3, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

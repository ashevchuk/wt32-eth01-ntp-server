#include "gnss_config.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "settings.h"
#include "ubx.h"

#define CLASS_CFG 0x06
#define ID_CFG_NAV5 0x24
#define ID_CFG_TP5 0x31
#define ID_CFG_CFG 0x09

#define NAV5_PAYLOAD_SIZE 36
#define NAV5_MASK_DYNMODEL 0x0001
#define NAV5_OFFSET_MASK 0
#define NAV5_OFFSET_DYNMODEL 2
#define DYNAMIC_MODEL_STATIONARY 2

#define TP5_PAYLOAD_SIZE 32
#define TP5_TIME_PULSE_INDEX 0
#define TP5_OFFSET_CABLE_DELAY 4
#define TP5_OFFSET_USER_DELAY 24

#define CFG_ALL_SECTIONS 0x0000FFFF
#define CFG_DEVICE_BBR_AND_FLASH 0x03
#define CFG_PAYLOAD_SIZE 13

#define START_DELAY_MS 3000
#define TASK_STACK_SIZE 4096
#define TASK_PRIORITY 3

static const char *TAG = "gnss_config";

static gnss_receiver_t receiver;
static portMUX_TYPE receiver_lock = portMUX_INITIALIZER_UNLOCKED;

static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

static void put_u32(uint8_t *p, uint32_t value)
{
    for (int i = 0; i < 4; i++) {
        p[i] = (uint8_t)(value >> (8 * i));
    }
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static esp_err_t poll_nav5(uint8_t payload[NAV5_PAYLOAD_SIZE])
{
    size_t length;
    ESP_RETURN_ON_ERROR(ubx_poll(CLASS_CFG, ID_CFG_NAV5, NULL, 0, payload, NAV5_PAYLOAD_SIZE, &length), TAG, "poll NAV5");
    return length == NAV5_PAYLOAD_SIZE ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

static esp_err_t poll_tp5(uint8_t payload[TP5_PAYLOAD_SIZE])
{
    const uint8_t request[] = {TP5_TIME_PULSE_INDEX};
    size_t length;
    ESP_RETURN_ON_ERROR(ubx_poll(CLASS_CFG, ID_CFG_TP5, request, sizeof(request), payload, TP5_PAYLOAD_SIZE, &length),
                        TAG, "poll TP5");
    return length == TP5_PAYLOAD_SIZE ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

static esp_err_t ensure_stationary(bool *changed)
{
    uint8_t nav5[NAV5_PAYLOAD_SIZE];
    ESP_RETURN_ON_ERROR(poll_nav5(nav5), TAG, "read NAV5");
    if (nav5[NAV5_OFFSET_DYNMODEL] != DYNAMIC_MODEL_STATIONARY) {
        put_u16(nav5 + NAV5_OFFSET_MASK, NAV5_MASK_DYNMODEL);
        nav5[NAV5_OFFSET_DYNMODEL] = DYNAMIC_MODEL_STATIONARY;
        ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_NAV5, nav5, sizeof(nav5)), TAG, "write NAV5");
        *changed = true;
    }
    return ESP_OK;
}

static esp_err_t ensure_delays(const pps_delay_settings_t *wanted, bool *changed)
{
    uint8_t tp5[TP5_PAYLOAD_SIZE];
    ESP_RETURN_ON_ERROR(poll_tp5(tp5), TAG, "read TP5");
    if (!wanted->configured) {
        return ESP_OK;
    }
    const int16_t cable = (int16_t)get_u16(tp5 + TP5_OFFSET_CABLE_DELAY);
    const int32_t user = (int32_t)get_u32(tp5 + TP5_OFFSET_USER_DELAY);
    if (cable != wanted->cable_delay_ns || user != wanted->user_delay_ns) {
        put_u16(tp5 + TP5_OFFSET_CABLE_DELAY, (uint16_t)wanted->cable_delay_ns);
        put_u32(tp5 + TP5_OFFSET_USER_DELAY, (uint32_t)wanted->user_delay_ns);
        ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_TP5, tp5, sizeof(tp5)), TAG, "write TP5");
        *changed = true;
    }
    return ESP_OK;
}

// Persisting costs flash cycles in the receiver, so it only happens after an actual change.
static esp_err_t save_to_receiver(void)
{
    uint8_t payload[CFG_PAYLOAD_SIZE] = {0};
    put_u32(payload + 4, CFG_ALL_SECTIONS);
    payload[12] = CFG_DEVICE_BBR_AND_FLASH;
    return ubx_set(CLASS_CFG, ID_CFG_CFG, payload, sizeof(payload));
}

static esp_err_t refresh_state(const pps_delay_settings_t *wanted, bool saved)
{
    uint8_t nav5[NAV5_PAYLOAD_SIZE], tp5[TP5_PAYLOAD_SIZE];
    ESP_RETURN_ON_ERROR(poll_nav5(nav5), TAG, "re-read NAV5");
    ESP_RETURN_ON_ERROR(poll_tp5(tp5), TAG, "re-read TP5");
    portENTER_CRITICAL(&receiver_lock);
    receiver.known = true;
    receiver.saved = receiver.saved || saved;
    receiver.dynamic_model = nav5[NAV5_OFFSET_DYNMODEL];
    receiver.cable_delay_ns = (int16_t)get_u16(tp5 + TP5_OFFSET_CABLE_DELAY);
    receiver.user_delay_ns = (int32_t)get_u32(tp5 + TP5_OFFSET_USER_DELAY);
    receiver.delays_managed = wanted->configured;
    portEXIT_CRITICAL(&receiver_lock);
    ESP_LOGI(TAG, "dynamic model %u, cable delay %d ns, user delay %ld ns", nav5[NAV5_OFFSET_DYNMODEL],
             (int16_t)get_u16(tp5 + TP5_OFFSET_CABLE_DELAY), (long)(int32_t)get_u32(tp5 + TP5_OFFSET_USER_DELAY));
    return ESP_OK;
}

static esp_err_t apply(bool *written)
{
    pps_delay_settings_t wanted;
    ESP_RETURN_ON_ERROR(settings_load_pps_delay(&wanted), TAG, "load delays");

    bool changed = false;
    ESP_RETURN_ON_ERROR(ensure_stationary(&changed), TAG, "dynamic model");
    ESP_RETURN_ON_ERROR(ensure_delays(&wanted, &changed), TAG, "delays");

    bool saved = false;
    *written = changed;
    if (changed) {
        ESP_RETURN_ON_ERROR(save_to_receiver(), TAG, "save to receiver memory");
        saved = true;
        ESP_LOGI(TAG, "receiver configuration changed and saved");
    } else {
        ESP_LOGI(TAG, "receiver already configured");
    }
    return refresh_state(&wanted, saved);
}

esp_err_t gnss_config_set_pps_delay(int16_t cable_delay_ns, int32_t user_delay_ns, bool *written)
{
    ESP_RETURN_ON_FALSE(cable_delay_ns >= 0 && user_delay_ns >= -GNSS_DELAY_USER_LIMIT_NS
                            && user_delay_ns <= GNSS_DELAY_USER_LIMIT_NS,
                        ESP_ERR_INVALID_ARG, TAG, "delay out of range");
    const pps_delay_settings_t wanted = {true, cable_delay_ns, user_delay_ns};
    ESP_RETURN_ON_ERROR(settings_save_pps_delay(&wanted), TAG, "store delays");
    return apply(written);
}

void gnss_config_get(gnss_receiver_t *out)
{
    portENTER_CRITICAL(&receiver_lock);
    *out = receiver;
    portEXIT_CRITICAL(&receiver_lock);
}

static void config_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));
    bool written;
    const esp_err_t err = apply(&written);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "receiver not configured: %s (UBX input may be disabled on its UART)", esp_err_to_name(err));
    }
    vTaskDelete(NULL);
}

esp_err_t gnss_config_start(void)
{
    ESP_RETURN_ON_FALSE(xTaskCreate(config_task, "gnss_config", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

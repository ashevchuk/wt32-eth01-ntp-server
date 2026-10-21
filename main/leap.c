#include "leap.h"

#include <stddef.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ntp_clock.h"
#include "ubx.h"

#define CLASS_AID 0x0B
#define ID_AID_HUI 0x02

#define HUI_PAYLOAD_SIZE 72
#define HUI_OFFSET_CURRENT_LS 26
#define HUI_OFFSET_EVENT_WEEK 28
#define HUI_OFFSET_EVENT_DAY 30
#define HUI_OFFSET_SCHEDULED_LS 32
#define HUI_OFFSET_FLAGS 68
#define HUI_FLAG_UTC_VALID 0x02

#define GPS_EPOCH_UNIX_S 315964800LL
#define SECONDS_PER_WEEK 604800LL
#define SECONDS_PER_DAY 86400LL
#define MICROS_PER_SECOND 1000000LL
#define START_DELAY_MS 10000
#define POLL_INTERVAL_MS 60000
#define TASK_STACK_SIZE 3072
#define TASK_PRIORITY 3

static const char *TAG = "leap";

static struct {
    bool known;
    int8_t gps_utc_offset_s;
    int8_t change;
    bool event_known;
    int64_t event_mono_us;
} state;
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;

uint8_t leap_indicator_at(int8_t change, int64_t unix_s, int64_t seconds_to_event)
{
    if (change == 0 || seconds_to_event <= 0) {
        return LEAP_NONE;
    }
    const int64_t event_s = unix_s + seconds_to_event;
    if (unix_s / SECONDS_PER_DAY != (event_s - 1) / SECONDS_PER_DAY) {
        return LEAP_NONE;
    }
    return change > 0 ? LEAP_INSERT : LEAP_DELETE;
}

void leap_get(leap_status_t *status)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&state_lock);
    status->known = state.known;
    status->gps_utc_offset_s = state.gps_utc_offset_s;
    status->change = state.change;
    status->event_known = state.event_known;
    status->seconds_to_event = (state.event_mono_us - now_us) / MICROS_PER_SECOND;
    portEXIT_CRITICAL(&state_lock);
}

uint8_t leap_warning(int64_t mono_us, int64_t unix_s)
{
    portENTER_CRITICAL(&state_lock);
    const bool pending = state.event_known;
    const int8_t change = state.change;
    const int64_t seconds_to_event = (state.event_mono_us - mono_us) / MICROS_PER_SECOND;
    portEXIT_CRITICAL(&state_lock);
    return pending ? leap_indicator_at(change, unix_s, seconds_to_event) : LEAP_NONE;
}

static int16_t get_i16(const uint8_t *p)
{
    return (int16_t)(p[0] | (p[1] << 8));
}

// The receiver's firmware (protocol 15) does not answer UBX-NAV-TIMELS, but UBX-AID-HUI carries the same
// information as broadcast by the satellites: the current and the scheduled offset and the week and day of the change.
static esp_err_t poll_receiver(void)
{
    uint8_t payload[HUI_PAYLOAD_SIZE];
    size_t length;
    ESP_RETURN_ON_ERROR(ubx_poll(CLASS_AID, ID_AID_HUI, NULL, 0, payload, sizeof(payload), &length), TAG, "poll");
    ESP_RETURN_ON_FALSE(length == HUI_PAYLOAD_SIZE, ESP_ERR_INVALID_SIZE, TAG, "reply size %u", (unsigned)length);

    const bool known = payload[HUI_OFFSET_FLAGS] & HUI_FLAG_UTC_VALID;
    const int8_t offset = (int8_t)get_i16(payload + HUI_OFFSET_CURRENT_LS);
    const int change = get_i16(payload + HUI_OFFSET_SCHEDULED_LS) - get_i16(payload + HUI_OFFSET_CURRENT_LS);
    const int8_t change_sign = change > 0 ? 1 : (change < 0 ? -1 : 0);

    // The change happens at the end of the given day of the GPS week (day 1 is Sunday).
    const int64_t event_unix_s = GPS_EPOCH_UNIX_S + (int64_t)get_i16(payload + HUI_OFFSET_EVENT_WEEK) * SECONDS_PER_WEEK
                                 + (int64_t)get_i16(payload + HUI_OFFSET_EVENT_DAY) * SECONDS_PER_DAY;
    const int64_t now_mono_us = esp_timer_get_time();
    ntp_clock_reading_t now;
    ntp_clock_read(now_mono_us, &now);
    const bool event_known = known && change_sign != 0 && now.source != NTP_CLOCK_UNSYNCHRONIZED;

    portENTER_CRITICAL(&state_lock);
    const bool differs = state.known != known || state.change != change_sign || state.gps_utc_offset_s != offset;
    state.known = known;
    state.gps_utc_offset_s = offset;
    state.change = change_sign;
    state.event_known = event_known;
    state.event_mono_us = event_known ? now_mono_us + (event_unix_s * MICROS_PER_SECOND - now.unix_us) : 0;
    portEXIT_CRITICAL(&state_lock);

    if (differs) {
        ESP_LOGI(TAG, "receiver %s, GPS-UTC %d s, scheduled change %d", known ? "knows the leap seconds" : "does not know the leap seconds yet",
                 offset, change_sign);
    }
    return ESP_OK;
}

static void leap_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));
    bool failing = false;
    while (true) {
        const esp_err_t err = poll_receiver();
        if (err != ESP_OK && !failing) {
            ESP_LOGW(TAG, "no leap second data: %s", esp_err_to_name(err));
        }
        failing = err != ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

esp_err_t leap_start(void)
{
    ESP_RETURN_ON_FALSE(xTaskCreate(leap_task, "leap", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

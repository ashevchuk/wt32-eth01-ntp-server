#include "ntp_clock.h"

#include <stdbool.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rtc.h"

#define MICROS_PER_SECOND 1000000LL
#define GNSS_HOLDOVER_US (10 * 60 * MICROS_PER_SECOND)
#define NETWORK_MAX_AGE_US (45 * 60 * MICROS_PER_SECOND)
#define RTC_REANCHOR_INTERVAL_US (60 * MICROS_PER_SECOND)
#define RTC_CHECK_INTERVAL_MS 1000
#define RTC_ROLLOVER_TIMEOUT_US (1500 * 1000LL)
// Lets other tasks (the LCD clock) get a turn on the shared I2C bus between two polls.
#define RTC_POLL_GAP_US 500

#define STRATUM_GNSS 1
#define STRATUM_LOCAL 10
#define STRATUM_UNSYNCHRONIZED 16
#define DISPERSION_GNSS_US 2000
#define DISPERSION_NETWORK_US 50000
#define DISPERSION_LOCAL_US 20000

enum anchor_kind { ANCHOR_GNSS, ANCHOR_NETWORK, ANCHOR_RTC, ANCHOR_KIND_COUNT };

struct anchor {
    bool valid;
    int64_t unix_us;
    int64_t mono_us;
    uint8_t stratum;
    uint8_t reference_id[4];
    int64_t root_delay_us;
};

static const char *TAG = "ntp_clock";

static struct anchor anchors[ANCHOR_KIND_COUNT];
static portMUX_TYPE anchor_lock = portMUX_INITIALIZER_UNLOCKED;

static void store_anchor(enum anchor_kind kind, const struct anchor *anchor)
{
    portENTER_CRITICAL(&anchor_lock);
    anchors[kind] = *anchor;
    portEXIT_CRITICAL(&anchor_lock);
}

static void load_anchors(struct anchor copy[ANCHOR_KIND_COUNT])
{
    portENTER_CRITICAL(&anchor_lock);
    memcpy(copy, anchors, sizeof(anchors));
    portEXIT_CRITICAL(&anchor_lock);
}

static bool fresh(const struct anchor *a, int64_t now_us, int64_t max_age_us)
{
    return a->valid && now_us - a->mono_us < max_age_us;
}

void ntp_clock_anchor_gnss(time_t unix_s, int64_t edge_us)
{
    store_anchor(ANCHOR_GNSS, &(struct anchor){.valid = true, .unix_us = (int64_t)unix_s * MICROS_PER_SECOND,
                                               .mono_us = edge_us});
}

void ntp_clock_anchor_local(time_t unix_s, int64_t mono_us)
{
    store_anchor(ANCHOR_RTC, &(struct anchor){.valid = true, .unix_us = (int64_t)unix_s * MICROS_PER_SECOND,
                                              .mono_us = mono_us});
}

void ntp_clock_anchor_network(int64_t unix_us, int64_t mono_us, uint8_t stratum, const uint8_t reference_id[4],
                              int64_t root_delay_us)
{
    struct anchor anchor = {.valid = true, .unix_us = unix_us, .mono_us = mono_us, .stratum = stratum,
                            .root_delay_us = root_delay_us};
    memcpy(anchor.reference_id, reference_id, sizeof(anchor.reference_id));
    store_anchor(ANCHOR_NETWORK, &anchor);
}

// GNSS wins while it is fresh, then the network. Otherwise the newest anchor of any kind is extrapolated.
void ntp_clock_read(int64_t mono_us, ntp_clock_reading_t *reading)
{
    struct anchor all[ANCHOR_KIND_COUNT];
    load_anchors(all);

    const struct anchor *chosen = NULL;
    *reading = (ntp_clock_reading_t){.source = NTP_CLOCK_UNSYNCHRONIZED, .stratum = STRATUM_UNSYNCHRONIZED};
    if (fresh(&all[ANCHOR_GNSS], mono_us, GNSS_HOLDOVER_US)) {
        chosen = &all[ANCHOR_GNSS];
        reading->source = NTP_CLOCK_GNSS;
        reading->stratum = STRATUM_GNSS;
        memcpy(reading->reference_id, "GPS", 4);
        reading->root_dispersion_us = DISPERSION_GNSS_US;
    } else if (fresh(&all[ANCHOR_NETWORK], mono_us, NETWORK_MAX_AGE_US)) {
        chosen = &all[ANCHOR_NETWORK];
        reading->source = NTP_CLOCK_NETWORK;
        reading->stratum = chosen->stratum;
        memcpy(reading->reference_id, chosen->reference_id, sizeof(reading->reference_id));
        reading->root_delay_us = chosen->root_delay_us;
        reading->root_dispersion_us = DISPERSION_NETWORK_US;
    } else {
        for (int kind = 0; kind < ANCHOR_KIND_COUNT; kind++) {
            if (all[kind].valid && (!chosen || all[kind].mono_us > chosen->mono_us)) {
                chosen = &all[kind];
            }
        }
        if (chosen) {
            reading->source = NTP_CLOCK_LOCAL;
            reading->stratum = STRATUM_LOCAL;
            memcpy(reading->reference_id, "LOCL", 4);
            reading->root_dispersion_us = DISPERSION_LOCAL_US;
        }
    }
    if (chosen) {
        reading->reference_unix_us = chosen->unix_us;
        reading->unix_us = chosen->unix_us + (mono_us - chosen->mono_us);
    }
}

static bool rtc_anchor_due(void)
{
    struct anchor all[ANCHOR_KIND_COUNT];
    load_anchors(all);
    const int64_t now_us = esp_timer_get_time();
    if (fresh(&all[ANCHOR_GNSS], now_us, GNSS_HOLDOVER_US) || fresh(&all[ANCHOR_NETWORK], now_us, NETWORK_MAX_AGE_US)) {
        return false;
    }
    return !all[ANCHOR_RTC].valid || now_us - all[ANCHOR_RTC].mono_us >= RTC_REANCHOR_INTERVAL_US;
}

// The DS3231 has one-second resolution, so the anchor is taken at the instant its seconds register changes.
static void anchor_to_rtc_second(void)
{
    struct tm before;
    if (rtc_read(&before) != ESP_OK) {
        return;
    }
    const int64_t start_us = esp_timer_get_time();
    struct tm now;
    while (esp_timer_get_time() - start_us < RTC_ROLLOVER_TIMEOUT_US) {
        if (rtc_read(&now) != ESP_OK) {
            return;
        }
        if (now.tm_sec != before.tm_sec) {
            ntp_clock_anchor_local(timegm(&now), esp_timer_get_time());
            ESP_LOGI(TAG, "anchored to rtc");
            return;
        }
        esp_rom_delay_us(RTC_POLL_GAP_US);
    }
}

static void rtc_fallback_task(void *arg)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_add(NULL));
    while (true) {
        esp_task_wdt_reset();
        if (rtc_anchor_due()) {
            anchor_to_rtc_second();
        }
        vTaskDelay(pdMS_TO_TICKS(RTC_CHECK_INTERVAL_MS));
    }
}

esp_err_t ntp_clock_start(void)
{
    ESP_RETURN_ON_FALSE(xTaskCreate(rtc_fallback_task, "ntp_rtc", 4096, NULL, 3, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

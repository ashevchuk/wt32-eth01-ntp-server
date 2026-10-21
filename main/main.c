#include <stdbool.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "bigclock.h"
#include "eth.h"
#include "gnss.h"
#include "gnss_config.h"
#include "i2c_bus.h"
#include "lcd.h"
#include "leap.h"
#include "munin.h"
#include "ntp_clock.h"
#include "ntp_server.h"
#include "ota.h"
#include "reset_button.h"
#include "rtc.h"
#include "sensors.h"
#include "settings.h"
#include "startup.h"
#include "upstream.h"
#include "web.h"
#include "wifi.h"

#define RTC_RESYNC_INTERVAL_US (3600LL * 1000000)
#define RTC_RETRY_INTERVAL_US (60LL * 1000000)

static const char *TAG = "ntp";

static void on_gnss_second(const struct tm *utc, int64_t edge_us)
{
    struct tm normalized = *utc;
    ntp_clock_anchor_gnss(timegm(&normalized), edge_us);

    static int64_t last_attempt_us;
    static bool synced;
    const int64_t now_us = esp_timer_get_time();
    if (last_attempt_us && now_us - last_attempt_us < (synced ? RTC_RESYNC_INTERVAL_US : RTC_RETRY_INTERVAL_US)) {
        return;
    }
    last_attempt_us = now_us;
    const esp_err_t err = rtc_write(utc);
    synced = err == ESP_OK;
    if (!synced) {
        ESP_LOGE(TAG, "rtc write failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "rtc set to %04d-%02d-%02d %02d:%02d:%02d", utc->tm_year + 1900, utc->tm_mon + 1,
             utc->tm_mday, utc->tm_hour, utc->tm_min, utc->tm_sec);
}

// A component that fails to start must not take the rest down: the device would reboot in a loop, never reach the
// OTA confirmation and have the bootloader roll back a good image.
static bool started(const char *component, esp_err_t err)
{
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s did not start: %s", component, esp_err_to_name(err));
        startup_failed(component);
    }
    return err == ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "NTP server starting");
    ESP_ERROR_CHECK(settings_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(settings_record_reset(esp_reset_reason()));

    // The defaults stay in place when the stored settings cannot be read.
    network_settings_t network;
    started("network settings", settings_load_network(&network));
    started("ethernet", eth_start(&network));
    started("wifi", wifi_start(&network));

    i2c_master_bus_handle_t i2c_bus;
    if (started("i2c bus", i2c_bus_init(&i2c_bus))) {
        const bool lcd_ready = started("lcd", lcd_start(i2c_bus));
        started("rtc", rtc_start(i2c_bus));
        if (lcd_ready) {
            started("clock display", bigclock_start());
        }
    }
    started("ntp clock", ntp_clock_start());
    if (started("gnss", gnss_start(on_gnss_second))) {
        started("gnss configuration", gnss_config_start());
        started("leap seconds", leap_start());
    }
    started("ntp server", ntp_server_start());
    started("sensors", sensors_start());
    started("reset button", reset_button_start());
    started("upstream client", upstream_start());
    started("munin node", munin_start());
    started("web server", web_start());
    ESP_ERROR_CHECK_WITHOUT_ABORT(ota_confirm_running_image());
}

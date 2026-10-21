#include <stdbool.h>

#include "esp_check.h"
#include "esp_log.h"
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
#include "upstream.h"
#include "web.h"
#include "wifi.h"

#define RTC_RESYNC_INTERVAL_US (3600LL * 1000000)

static const char *TAG = "ntp";

static void on_gnss_second(const struct tm *utc, int64_t edge_us)
{
    struct tm normalized = *utc;
    ntp_clock_anchor_gnss(timegm(&normalized), edge_us);

    static int64_t last_sync_us;
    static bool synced;
    int64_t now_us = esp_timer_get_time();
    if (synced && now_us - last_sync_us < RTC_RESYNC_INTERVAL_US) {
        return;
    }
    esp_err_t err = rtc_write(utc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rtc write failed: %s", esp_err_to_name(err));
        return;
    }
    synced = true;
    last_sync_us = now_us;
    ESP_LOGI(TAG, "rtc set to %04d-%02d-%02d %02d:%02d:%02d", utc->tm_year + 1900, utc->tm_mon + 1,
             utc->tm_mday, utc->tm_hour, utc->tm_min, utc->tm_sec);
}

void app_main(void)
{
    ESP_LOGI(TAG, "NTP server starting");
    ESP_ERROR_CHECK(settings_init());

    network_settings_t network;
    ESP_ERROR_CHECK(settings_load_network(&network));
    ESP_ERROR_CHECK(eth_start(&network));
    ESP_ERROR_CHECK(wifi_start(&network));

    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_bus_init(&i2c_bus));
    ESP_ERROR_CHECK(lcd_start(i2c_bus));
    ESP_ERROR_CHECK(rtc_start(i2c_bus));
    ESP_ERROR_CHECK(bigclock_start());
    ESP_ERROR_CHECK(ntp_clock_start());
    ESP_ERROR_CHECK(gnss_start(on_gnss_second));
    ESP_ERROR_CHECK(gnss_config_start());
    ESP_ERROR_CHECK(leap_start());
    ESP_ERROR_CHECK(ntp_server_start());
    ESP_ERROR_CHECK(sensors_start());
    ESP_ERROR_CHECK(reset_button_start());
    ESP_ERROR_CHECK(upstream_start());
    ESP_ERROR_CHECK(munin_start());
    ESP_ERROR_CHECK(web_start());
    ESP_ERROR_CHECK(ota_confirm_running_image());
}

#include "wifi.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

// Ethernet is the primary interface; the default route priority of Wi-Fi (100) would otherwise win over it (50).
#define WIFI_ROUTE_PRIORITY 10

static const char *TAG = "wifi";

static bool enabled;
static bool connected;
static esp_netif_t *sta_netif;

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED) {
        connected = false;
        esp_wifi_connect();
    }
}

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const ip_event_got_ip_t *event = data;
    connected = true;
    ESP_LOGI(TAG, "ip " IPSTR, IP2STR(&event->ip_info.ip));
}

esp_err_t wifi_start(const network_settings_t *network)
{
    if (!network->wifi_enabled) {
        return ESP_OK;
    }
    sta_netif = esp_netif_create_default_wifi_sta();
    ESP_RETURN_ON_FALSE(sta_netif, ESP_ERR_NO_MEM, TAG, "netif");
    esp_netif_set_route_prio(sta_netif, WIFI_ROUTE_PRIORITY);
    ESP_RETURN_ON_ERROR(esp_netif_set_hostname(sta_netif, network->hostname), TAG, "hostname");

    const wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL), TAG, "wifi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_got_ip, NULL), TAG, "ip handler");

    wifi_config_t config = {0};
    strlcpy((char *)config.sta.ssid, network->wifi_ssid, sizeof(config.sta.ssid));
    strlcpy((char *)config.sta.password, network->wifi_password, sizeof(config.sta.password));
    config.sta.threshold.authmode = network->wifi_password[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &config), TAG, "config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start");
    enabled = true;
    return ESP_OK;
}

void wifi_get_status(wifi_status_t *status)
{
    *status = (wifi_status_t){.enabled = enabled, .connected = connected};
    strcpy(status->ip, "-");
    if (!connected) {
        return;
    }
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        status->rssi = ap.rssi;
    }
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(sta_netif, &info) == ESP_OK) {
        snprintf(status->ip, sizeof(status->ip), IPSTR, IP2STR(&info.ip));
    }
}

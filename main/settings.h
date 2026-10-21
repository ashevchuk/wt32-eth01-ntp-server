#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define SETTINGS_TIMEZONE_MAX_LENGTH 64

// Initializes NVS and applies the stored timezone to the C library.
esp_err_t settings_init(void);

// A POSIX TZ string such as "EET-2EEST,M3.5.0/3,M10.5.0/4".
bool settings_timezone_valid(const char *timezone);
esp_err_t settings_get_timezone(char timezone[SETTINGS_TIMEZONE_MAX_LENGTH]);
esp_err_t settings_set_timezone(const char *timezone);

#define SETTINGS_IP_LENGTH 16
#define SETTINGS_SSID_MAX_LENGTH 33
#define SETTINGS_WIFI_PASSWORD_MAX_LENGTH 64
#define SETTINGS_HOSTNAME_MAX_LENGTH 32

typedef struct {
    bool eth_static;
    char eth_ip[SETTINGS_IP_LENGTH];
    char eth_mask[SETTINGS_IP_LENGTH];
    char eth_gateway[SETTINGS_IP_LENGTH];
    char eth_dns[SETTINGS_IP_LENGTH];
    bool wifi_enabled;
    char wifi_ssid[SETTINGS_SSID_MAX_LENGTH];
    char wifi_password[SETTINGS_WIFI_PASSWORD_MAX_LENGTH];
    char hostname[SETTINGS_HOSTNAME_MAX_LENGTH];
} network_settings_t;

// Falls back to defaults (DHCP, Wi-Fi off) when nothing valid is stored.
esp_err_t settings_load_network(network_settings_t *network);
esp_err_t settings_save_network(const network_settings_t *network);

// Returns a human-readable reason, or NULL when the settings are acceptable.
const char *settings_network_error(const network_settings_t *network);

#define SETTINGS_MAX_SENSORS 16
#define SETTINGS_SENSOR_NAME_MAX_LENGTH 32
#define SETTINGS_ROM_LENGTH 8

typedef struct {
    uint8_t rom[SETTINGS_ROM_LENGTH];
    char name[SETTINGS_SENSOR_NAME_MAX_LENGTH];
} sensor_name_t;

// Unused slots have an empty name.
esp_err_t settings_load_sensor_names(sensor_name_t names[SETTINGS_MAX_SENSORS]);
esp_err_t settings_save_sensor_names(const sensor_name_t names[SETTINGS_MAX_SENSORS]);

#define SETTINGS_MUNIN_PLUGIN_MAX_LENGTH SETTINGS_SENSOR_NAME_MAX_LENGTH

// Defaults to "environment" when nothing is stored.
esp_err_t settings_load_munin_plugin(char plugin[SETTINGS_MUNIN_PLUGIN_MAX_LENGTH]);
esp_err_t settings_save_munin_plugin(const char *plugin);

// Delays that the GNSS receiver compensates in its time pulse; when not configured the receiver keeps its own.
typedef struct {
    bool configured;
    int16_t cable_delay_ns;
    int32_t user_delay_ns;
} pps_delay_settings_t;

esp_err_t settings_load_pps_delay(pps_delay_settings_t *delay);
esp_err_t settings_save_pps_delay(const pps_delay_settings_t *delay);

#define SETTINGS_UPSTREAM_MAX_LENGTH 64

// Host name or IPv4 address of the fallback NTP server; empty means "off". Defaults to pool.ntp.org.
esp_err_t settings_load_upstream(char server[SETTINGS_UPSTREAM_MAX_LENGTH]);
esp_err_t settings_save_upstream(const char *server);

#include "settings.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "esp_check.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"

#define NAMESPACE "ntp"
#define KEY_TIMEZONE "timezone"
#define DEFAULT_TIMEZONE "UTC0"
#define TIMEZONE_MIN_LENGTH 3
#define TIMEZONE_ALLOWED_PUNCTUATION "+-:,./<>"
#define KEY_NETWORK "network"
#define KEY_SENSORS "sensors"
#define KEY_MUNIN_PLUGIN "munin_plugin"
#define KEY_PPS_DELAY "pps_delay"
#define KEY_UPSTREAM "upstream"
#define KEY_RESETS "resets"
#define KEY_MUNIN_SENSORS "munin_sensors"
#define DEFAULT_UPSTREAM "pool.ntp.org"
#define DEFAULT_MUNIN_PLUGIN "environment"
#define DEFAULT_HOSTNAME "ntp-server"
#define SSID_MAX_LENGTH 32
#define WPA_PASSPHRASE_MIN_LENGTH 8
#define WPA_PASSPHRASE_MAX_LENGTH 63
#define HOSTNAME_MAX_LENGTH 31

static const char *TAG = "settings";

static void apply_timezone(const char *timezone)
{
    setenv("TZ", timezone, 1);
    tzset();
}

bool settings_timezone_valid(const char *timezone)
{
    const size_t length = strlen(timezone);
    if (length < TIMEZONE_MIN_LENGTH || length >= SETTINGS_TIMEZONE_MAX_LENGTH) {
        return false;
    }
    if (!isalpha((unsigned char)timezone[0]) && timezone[0] != '<') {
        return false;
    }
    bool has_digit = false;
    for (size_t i = 0; i < length; i++) {
        const unsigned char c = (unsigned char)timezone[i];
        has_digit |= isdigit(c) != 0;
        if (!isalnum(c) && !strchr(TIMEZONE_ALLOWED_PUNCTUATION, c)) {
            return false;
        }
    }
    return has_digit;
}

esp_err_t settings_get_timezone(char timezone[SETTINGS_TIMEZONE_MAX_LENGTH])
{
    nvs_handle_t handle;
    size_t size = SETTINGS_TIMEZONE_MAX_LENGTH;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        err = nvs_get_str(handle, KEY_TIMEZONE, timezone, &size);
        nvs_close(handle);
    }
    if (err == ESP_ERR_NVS_NOT_FOUND || (err == ESP_OK && !settings_timezone_valid(timezone))) {
        strcpy(timezone, DEFAULT_TIMEZONE);
        return ESP_OK;
    }
    return err;
}

esp_err_t settings_set_timezone(const char *timezone)
{
    ESP_RETURN_ON_FALSE(settings_timezone_valid(timezone), ESP_ERR_INVALID_ARG, TAG, "invalid timezone");
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(NAMESPACE, NVS_READWRITE, &handle), TAG, "open");
    esp_err_t err = nvs_set_str(handle, KEY_TIMEZONE, timezone);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    ESP_RETURN_ON_ERROR(err, TAG, "store timezone");
    apply_timezone(timezone);
    return ESP_OK;
}

esp_err_t settings_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "erase");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs init");

    char timezone[SETTINGS_TIMEZONE_MAX_LENGTH];
    ESP_RETURN_ON_ERROR(settings_get_timezone(timezone), TAG, "read timezone");
    apply_timezone(timezone);
    return ESP_OK;
}

static void network_defaults(network_settings_t *network)
{
    *network = (network_settings_t){0};
    strcpy(network->hostname, DEFAULT_HOSTNAME);
}

esp_err_t settings_load_network(network_settings_t *network)
{
    network_defaults(network);
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "open");

    network_settings_t stored;
    size_t size = sizeof(stored);
    err = nvs_get_blob(handle, KEY_NETWORK, &stored, &size);
    nvs_close(handle);
    if (err == ESP_OK && size == sizeof(stored) && settings_network_error(&stored) == NULL) {
        *network = stored;
        return ESP_OK;
    }
    return (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) ? ESP_OK : err;
}

esp_err_t settings_save_network(const network_settings_t *network)
{
    ESP_RETURN_ON_FALSE(settings_network_error(network) == NULL, ESP_ERR_INVALID_ARG, TAG, "invalid network settings");
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(NAMESPACE, NVS_READWRITE, &handle), TAG, "open");
    esp_err_t err = nvs_set_blob(handle, KEY_NETWORK, network, sizeof(*network));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static bool parse_ip(const char *text, uint32_t *host_order)
{
    struct in_addr address;
    if (inet_pton(AF_INET, text, &address) != 1) {
        return false;
    }
    *host_order = ntohl(address.s_addr);
    return true;
}

static bool is_netmask(uint32_t mask)
{
    const uint32_t host_bits = ~mask;
    return mask != 0 && (host_bits & (host_bits + 1)) == 0;
}

static bool printable_ascii(const char *text)
{
    for (; *text; text++) {
        if (*text < ' ' || *text > '~') {
            return false;
        }
    }
    return true;
}

static bool hostname_valid(const char *name)
{
    const size_t length = strlen(name);
    if (length == 0 || length > HOSTNAME_MAX_LENGTH || name[0] == '-' || name[length - 1] == '-') {
        return false;
    }
    for (size_t i = 0; i < length; i++) {
        if (!isalnum((unsigned char)name[i]) && name[i] != '-') {
            return false;
        }
    }
    return true;
}

static const char *static_address_error(const char *ip_text, const char *mask_text, const char *gateway_text,
                                        const char *dns_text)
{
    uint32_t ip, mask, gateway, dns;
    if (!parse_ip(ip_text, &ip) || ip == 0) {
        return "invalid IP address";
    }
    if (!parse_ip(mask_text, &mask) || !is_netmask(mask)) {
        return "invalid netmask";
    }
    if (gateway_text[0] && (!parse_ip(gateway_text, &gateway) || (gateway & mask) != (ip & mask))) {
        return "gateway is invalid or outside the subnet";
    }
    if (dns_text[0] && !parse_ip(dns_text, &dns)) {
        return "invalid DNS address";
    }
    return NULL;
}

const char *settings_network_error(const network_settings_t *network)
{
    if (!hostname_valid(network->hostname)) {
        return "invalid hostname";
    }
    if (network->eth_static) {
        const char *error = static_address_error(network->eth_ip, network->eth_mask, network->eth_gateway,
                                                 network->eth_dns);
        if (error) {
            return error;
        }
    }
    if (network->wifi_enabled) {
        const size_t ssid_length = strlen(network->wifi_ssid);
        const size_t password_length = strlen(network->wifi_password);
        if (ssid_length == 0 || ssid_length > SSID_MAX_LENGTH || !printable_ascii(network->wifi_ssid)) {
            return "SSID must be 1-32 printable ASCII characters";
        }
        if (password_length != 0 && (password_length < WPA_PASSPHRASE_MIN_LENGTH
                                     || password_length > WPA_PASSPHRASE_MAX_LENGTH
                                     || !printable_ascii(network->wifi_password))) {
            return "Wi-Fi password must be empty or 8-63 printable ASCII characters";
        }
        if (network->wifi_static) {
            const char *error = static_address_error(network->wifi_ip, network->wifi_mask, network->wifi_gateway,
                                                     network->wifi_dns);
            if (error) {
                return error;
            }
        }
    }
    return NULL;
}

esp_err_t settings_load_munin_sensors(uint8_t roms[SETTINGS_MAX_SENSORS][SETTINGS_ROM_LENGTH])
{
    memset(roms, 0, SETTINGS_MAX_SENSORS * SETTINGS_ROM_LENGTH);
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "open");

    uint8_t saved[SETTINGS_MAX_SENSORS][SETTINGS_ROM_LENGTH];
    size_t size = sizeof(saved);
    err = nvs_get_blob(handle, KEY_MUNIN_SENSORS, saved, &size);
    nvs_close(handle);
    if (err == ESP_OK && size == sizeof(saved)) {
        memcpy(roms, saved, sizeof(saved));
        return ESP_OK;
    }
    return err == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : err;
}

esp_err_t settings_save_munin_sensors(const uint8_t roms[SETTINGS_MAX_SENSORS][SETTINGS_ROM_LENGTH])
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(NAMESPACE, NVS_READWRITE, &handle), TAG, "open");
    esp_err_t err = nvs_set_blob(handle, KEY_MUNIN_SENSORS, roms, SETTINGS_MAX_SENSORS * SETTINGS_ROM_LENGTH);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t settings_load_sensor_names(sensor_name_t names[SETTINGS_MAX_SENSORS])
{
    memset(names, 0, sizeof(sensor_name_t) * SETTINGS_MAX_SENSORS);
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "open");

    sensor_name_t stored[SETTINGS_MAX_SENSORS];
    size_t size = sizeof(stored);
    err = nvs_get_blob(handle, KEY_SENSORS, stored, &size);
    nvs_close(handle);
    if (err == ESP_OK && size == sizeof(stored)) {
        memcpy(names, stored, sizeof(stored));
        return ESP_OK;
    }
    return err == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : err;
}

esp_err_t settings_save_sensor_names(const sensor_name_t names[SETTINGS_MAX_SENSORS])
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(NAMESPACE, NVS_READWRITE, &handle), TAG, "open");
    esp_err_t err = nvs_set_blob(handle, KEY_SENSORS, names, sizeof(sensor_name_t) * SETTINGS_MAX_SENSORS);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t settings_load_munin_plugin(char plugin[SETTINGS_MUNIN_PLUGIN_MAX_LENGTH])
{
    nvs_handle_t handle;
    size_t size = SETTINGS_MUNIN_PLUGIN_MAX_LENGTH;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        err = nvs_get_str(handle, KEY_MUNIN_PLUGIN, plugin, &size);
        nvs_close(handle);
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        strcpy(plugin, DEFAULT_MUNIN_PLUGIN);
        return ESP_OK;
    }
    return err;
}

esp_err_t settings_save_munin_plugin(const char *plugin)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(NAMESPACE, NVS_READWRITE, &handle), TAG, "open");
    esp_err_t err = nvs_set_str(handle, KEY_MUNIN_PLUGIN, plugin);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t settings_load_reset_counts(uint32_t counts[SETTINGS_RESET_REASONS])
{
    memset(counts, 0, sizeof(uint32_t) * SETTINGS_RESET_REASONS);
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "open");
    uint32_t stored[SETTINGS_RESET_REASONS];
    size_t size = sizeof(stored);
    err = nvs_get_blob(handle, KEY_RESETS, stored, &size);
    nvs_close(handle);
    if (err == ESP_OK && size == sizeof(stored)) {
        memcpy(counts, stored, sizeof(stored));
        return ESP_OK;
    }
    return err == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : err;
}

esp_err_t settings_record_reset(unsigned reason)
{
    uint32_t counts[SETTINGS_RESET_REASONS];
    ESP_RETURN_ON_ERROR(settings_load_reset_counts(counts), TAG, "load reset counts");
    counts[reason < SETTINGS_RESET_REASONS ? reason : 0]++;
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(NAMESPACE, NVS_READWRITE, &handle), TAG, "open");
    esp_err_t err = nvs_set_blob(handle, KEY_RESETS, counts, sizeof(counts));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t settings_clear_reset_counts(void)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(NAMESPACE, NVS_READWRITE, &handle), TAG, "open");
    esp_err_t err = nvs_erase_key(handle, KEY_RESETS);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t settings_load_pps_delay(pps_delay_settings_t *delay)
{
    *delay = (pps_delay_settings_t){0};
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "open");
    pps_delay_settings_t stored;
    size_t size = sizeof(stored);
    err = nvs_get_blob(handle, KEY_PPS_DELAY, &stored, &size);
    nvs_close(handle);
    if (err == ESP_OK && size == sizeof(stored)) {
        *delay = stored;
        return ESP_OK;
    }
    return err == ESP_ERR_NVS_NOT_FOUND ? ESP_OK : err;
}

esp_err_t settings_save_pps_delay(const pps_delay_settings_t *delay)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(NAMESPACE, NVS_READWRITE, &handle), TAG, "open");
    esp_err_t err = nvs_set_blob(handle, KEY_PPS_DELAY, delay, sizeof(*delay));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t settings_load_upstream(char server[SETTINGS_UPSTREAM_MAX_LENGTH])
{
    nvs_handle_t handle;
    size_t size = SETTINGS_UPSTREAM_MAX_LENGTH;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        err = nvs_get_str(handle, KEY_UPSTREAM, server, &size);
        nvs_close(handle);
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        strcpy(server, DEFAULT_UPSTREAM);
        return ESP_OK;
    }
    return err;
}

esp_err_t settings_save_upstream(const char *server)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(NAMESPACE, NVS_READWRITE, &handle), TAG, "open");
    esp_err_t err = nvs_set_str(handle, KEY_UPSTREAM, server);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

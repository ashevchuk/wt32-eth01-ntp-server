#include "web.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gnss.h"
#include "eth.h"
#include "gnss_config.h"
#include "leap.h"
#include "munin.h"
#include "ntp_clock.h"
#include "ntp_server.h"
#include "ota.h"
#include "rtc.h"
#include "sensors.h"
#include "settings.h"
#include "upstream.h"
#include "wifi.h"

#define STATUS_BUFFER_SIZE 1536
#define MICROS_PER_SECOND 1000000LL
#define ETH_NETIF_KEY "ETH_DEF"
#define MAC_LENGTH 6
#define TIMESTAMP_LENGTH 21
#define NO_ADDRESS "-"
#define IP_TEXT_LENGTH 16
#define MAC_TEXT_LENGTH 18
#define HTTP_STACK_SIZE 8192
#define HTTP_MAX_URI_HANDLERS 16
// The lwIP socket pool is shared with the HTTP listener and control socket, NTP, Munin and the SNTP client.
#define HTTP_MAX_OPEN_SOCKETS 8
#define HTTP_STATUS_CONFLICT "409 Conflict"
#define FORM_BUFFER_SIZE 768
#define PARAM_SIZE 200
#define REBOOT_DELAY_MS 500
#define LOCAL_TIMESTAMP_LENGTH 20
#define RTC_YEAR_MIN 2000
#define RTC_YEAR_MAX 2099
#define TM_YEAR_BASE 1900

extern const char index_html_start[] asm("_binary_index_html_start");

static const char *TAG = "web";

struct json_out {
    char buffer[STATUS_BUFFER_SIZE];
    size_t length;
};

static void json_printf(struct json_out *out, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    const int written = vsnprintf(out->buffer + out->length, sizeof(out->buffer) - out->length, format, args);
    va_end(args);
    if (written > 0) {
        const size_t limit = sizeof(out->buffer) - 1;
        out->length = (out->length + (size_t)written < limit) ? out->length + (size_t)written : limit;
    }
}

static const char *source_name(ntp_clock_source_t source)
{
    switch (source) {
    case NTP_CLOCK_GNSS:
        return "gnss";
    case NTP_CLOCK_NETWORK:
        return "network";
    case NTP_CLOCK_LOCAL:
        return "local";
    default:
        return "unsynchronized";
    }
}

static void format_iso8601(const struct tm *utc, char text[TIMESTAMP_LENGTH])
{
    strftime(text, TIMESTAMP_LENGTH, "%Y-%m-%dT%H:%M:%SZ", utc);
}

static void write_time(struct json_out *out)
{
    ntp_clock_reading_t reading;
    ntp_clock_read(esp_timer_get_time(), &reading);

    json_printf(out, "\"time\":{\"source\":\"%s\",\"stratum\":%u,", source_name(reading.source),
                reading.stratum);
    if (reading.source == NTP_CLOCK_UNSYNCHRONIZED) {
        json_printf(out, "\"utc\":null,\"local\":null,");
    } else {
        const time_t seconds = reading.unix_us / MICROS_PER_SECOND;
        struct tm utc;
        char text[TIMESTAMP_LENGTH];
        gmtime_r(&seconds, &utc);
        format_iso8601(&utc, text);
        json_printf(out, "\"utc\":\"%s\",", text);

        struct tm local;
        char local_text[LOCAL_TIMESTAMP_LENGTH];
        localtime_r(&seconds, &local);
        strftime(local_text, sizeof(local_text), "%Y-%m-%dT%H:%M:%S", &local);
        json_printf(out, "\"local\":\"%s\",", local_text);
    }

    char timezone[SETTINGS_TIMEZONE_MAX_LENGTH] = "";
    settings_get_timezone(timezone);
    json_printf(out, "\"timezone\":\"%s\",", timezone);

    struct tm rtc;
    const bool rtc_valid = rtc_read(&rtc) == ESP_OK;
    if (rtc_valid) {
        char text[TIMESTAMP_LENGTH];
        format_iso8601(&rtc, text);
        json_printf(out, "\"rtc\":\"%s\",", text);
    }
    json_printf(out, "\"rtc_valid\":%s,", rtc_valid ? "true" : "false");

    upstream_status_t upstream;
    upstream_get_status(&upstream);
    json_printf(out, "\"upstream\":{\"server\":\"%s\",\"enabled\":%s,\"synced\":%s,\"stratum\":%u,"
                     "\"age_s\":%lld,\"delay_ms\":%.1f,\"offset_ms\":",
                upstream.server, upstream.enabled ? "true" : "false", upstream.synced ? "true" : "false",
                upstream.server_stratum, (long long)upstream.age_s, upstream.delay_us / 1000.0);
    if (upstream.has_offset) {
        json_printf(out, "%.1f}}", upstream.offset_us / 1000.0);
    } else {
        json_printf(out, "null}}");
    }
}

static void write_gnss(struct json_out *out)
{
    gnss_status_t gnss;
    gnss_get_status(&gnss);
    json_printf(out,
                "\"gnss\":{\"fix\":%s,\"fix_mode\":%u,\"fix_quality\":%u,\"satellites_used\":%u,"
                "\"satellites_in_view\":%u,\"has_position\":%s,\"latitude\":%.7f,\"longitude\":%.7f,"
                "\"altitude\":%.1f,\"hdop\":%.2f,\"pdop\":%.2f,\"speed\":%.2f,\"course\":%.1f,",
                gnss.fix ? "true" : "false", gnss.fix_mode, gnss.fix_quality, gnss.satellites_used,
                gnss.satellites_in_view, gnss.has_position ? "true" : "false", gnss.latitude_deg,
                gnss.longitude_deg, gnss.altitude_m, gnss.hdop, gnss.pdop, gnss.speed_kmh, gnss.course_deg);

    leap_status_t leap;
    leap_get(&leap);
    json_printf(out, "\"leap\":{\"known\":%s,\"gps_utc_offset_s\":%d,\"change\":%d,\"seconds_to_event\":%lld}}",
                leap.known ? "true" : "false", leap.gps_utc_offset_s, leap.event_known ? leap.change : 0,
                (long long)(leap.event_known ? leap.seconds_to_event : 0));
}

static void json_string(struct json_out *out, const char *text)
{
    json_printf(out, "\"");
    for (; *text; text++) {
        if (*text == '"' || *text == '\\') {
            json_printf(out, "\\");
        }
        json_printf(out, "%c", *text);
    }
    json_printf(out, "\"");
}

static void write_network(struct json_out *out)
{
    char ip[IP_TEXT_LENGTH] = NO_ADDRESS, mask[IP_TEXT_LENGTH] = NO_ADDRESS, gw[IP_TEXT_LENGTH] = NO_ADDRESS;
    char dns[IP_TEXT_LENGTH] = NO_ADDRESS, mac[MAC_TEXT_LENGTH] = NO_ADDRESS;
    const char *hostname = "";
    bool dhcp = true;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey(ETH_NETIF_KEY);
    if (netif) {
        esp_netif_ip_info_t info;
        esp_netif_dns_info_t dns_info;
        uint8_t mac_bytes[MAC_LENGTH];
        esp_netif_dhcp_status_t dhcp_status;
        if (esp_netif_get_ip_info(netif, &info) == ESP_OK) {
            snprintf(ip, sizeof(ip), IPSTR, IP2STR(&info.ip));
            snprintf(mask, sizeof(mask), IPSTR, IP2STR(&info.netmask));
            snprintf(gw, sizeof(gw), IPSTR, IP2STR(&info.gw));
        }
        if (esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info) == ESP_OK && dns_info.ip.u_addr.ip4.addr) {
            snprintf(dns, sizeof(dns), IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        }
        if (esp_netif_get_mac(netif, mac_bytes) == ESP_OK) {
            snprintf(mac, sizeof(mac), MACSTR, MAC2STR(mac_bytes));
        }
        if (esp_netif_dhcpc_get_status(netif, &dhcp_status) == ESP_OK) {
            dhcp = dhcp_status == ESP_NETIF_DHCP_STARTED;
        }
        esp_netif_get_hostname(netif, &hostname);
    }
    eth_diagnostics_t diag;
    eth_get_diagnostics(&diag);
    json_printf(out, "\"network\":{\"hostname\":");
    json_string(out, hostname);
    json_printf(out, ",\"eth\":{\"mode\":\"%s\",\"ip\":\"%s\",\"mask\":\"%s\",\"gw\":\"%s\",\"dns\":\"%s\",\"mac\":\"%s\","
                     "\"speed\":%lu,\"full_duplex\":%s,"
                     "\"symbol_errors\":%lu,\"rx_missed\":%lu,\"rx_fifo_overflow\":%lu}",
                dhcp ? "dhcp" : "static", ip, mask, gw, dns, mac, (unsigned long)diag.link_speed_mbps,
                diag.full_duplex ? "true" : "false", (unsigned long)diag.phy_symbol_errors,
                (unsigned long)diag.emac_missed_frames, (unsigned long)diag.emac_fifo_overflows);

    wifi_status_t wifi;
    wifi_get_status(&wifi);
    json_printf(out, ",\"wifi\":{\"enabled\":%s,\"connected\":%s,\"ip\":\"%s\",\"rssi\":%d}}",
                wifi.enabled ? "true" : "false", wifi.connected ? "true" : "false", wifi.ip, wifi.rssi);
}

static const char *reset_reason_name(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
        return "power-on";
    case ESP_RST_SW:
        return "software";
    case ESP_RST_PANIC:
        return "panic";
    case ESP_RST_INT_WDT:
        return "interrupt watchdog";
    case ESP_RST_TASK_WDT:
        return "task watchdog";
    case ESP_RST_WDT:
        return "other watchdog";
    case ESP_RST_BROWNOUT:
        return "brownout";
    case ESP_RST_EXT:
        return "external pin";
    case ESP_RST_DEEPSLEEP:
        return "deep sleep";
    default:
        return "unknown";
    }
}

static esp_err_t status_handler(httpd_req_t *req)
{
    static struct json_out out;
    out.length = 0;

    const esp_app_desc_t *app = esp_app_get_description();
    json_printf(&out, "{");
    write_time(&out);
    json_printf(&out, ",\"ntp\":{\"requests\":%lu},", (unsigned long)ntp_server_request_count());
    write_gnss(&out);
    json_printf(&out, ",");
    write_network(&out);
    json_printf(&out, ",\"system\":{\"version\":\"%s\",\"idf\":\"%s\",\"partition\":\"%s\",\"reset_reason\":\"%s\",\"uptime_s\":%lld,\"free_heap\":%lu}}",
                app->version, app->idf_ver, ota_running_partition_label(), reset_reason_name(), esp_timer_get_time() / MICROS_PER_SECOND,
                (unsigned long)esp_get_free_heap_size());

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out.buffer, out.length);
}

esp_err_t web_respond(httpd_req_t *req, const char *status, const char *message)
{
    char body[128];
    const bool ok = strcmp(status, HTTPD_200) == 0;
    snprintf(body, sizeof(body), "{\"ok\":%s,\"message\":\"%s\"}", ok ? "true" : "false", message);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static void url_decode(char *text)
{
    char *out = text;
    for (const char *in = text; *in; in++) {
        if (*in == '+') {
            *out++ = ' ';
        } else if (*in == '%' && isxdigit((unsigned char)in[1]) && isxdigit((unsigned char)in[2])) {
            const char hex[3] = {in[1], in[2], '\0'};
            *out++ = (char)strtol(hex, NULL, 16);
            in += 2;
        } else {
            *out++ = *in;
        }
    }
    *out = '\0';
}

static bool read_form(httpd_req_t *req, char form[FORM_BUFFER_SIZE])
{
    if (req->content_len == 0 || req->content_len >= FORM_BUFFER_SIZE) {
        return false;
    }
    size_t received = 0;
    while (received < req->content_len) {
        const int n = httpd_req_recv(req, form + received, req->content_len - received);
        if (n <= 0) {
            return false;
        }
        received += (size_t)n;
    }
    form[received] = '\0';
    return true;
}

static bool form_value(const char *form, const char *key, char value[PARAM_SIZE])
{
    if (httpd_query_key_value(form, key, value, PARAM_SIZE) != ESP_OK) {
        return false;
    }
    url_decode(value);
    return true;
}

static esp_err_t timezone_handler(httpd_req_t *req)
{
    char form[FORM_BUFFER_SIZE], timezone[PARAM_SIZE];
    if (!read_form(req, form) || !form_value(form, "timezone", timezone)) {
        return web_respond(req, HTTPD_400, "missing timezone");
    }
    if (!settings_timezone_valid(timezone)) {
        return web_respond(req, HTTPD_400, "invalid timezone");
    }
    if (settings_set_timezone(timezone) != ESP_OK) {
        return web_respond(req, HTTPD_500, "could not store timezone");
    }
    return web_respond(req, HTTPD_200, "timezone saved");
}

// Accepts "YYYY-MM-DDTHH:MM:SS" with an optional trailing Z, always UTC.
static bool parse_utc(const char *text, struct tm *utc)
{
    int year, month, day, hour, minute, second;
    if (sscanf(text, "%d-%d-%dT%d:%d:%d", &year, &month, &day, &hour, &minute, &second) != 6
        || year < RTC_YEAR_MIN || year > RTC_YEAR_MAX) {
        return false;
    }
    *utc = (struct tm){
        .tm_year = year - TM_YEAR_BASE, .tm_mon = month - 1, .tm_mday = day,
        .tm_hour = hour, .tm_min = minute, .tm_sec = second,
    };
    const struct tm requested = *utc;
    timegm(utc);
    // timegm normalizes out-of-range fields silently, so a changed field means the input was invalid.
    return utc->tm_year == requested.tm_year && utc->tm_mon == requested.tm_mon && utc->tm_mday == requested.tm_mday
           && utc->tm_hour == requested.tm_hour && utc->tm_min == requested.tm_min
           && utc->tm_sec == requested.tm_sec;
}

static esp_err_t time_handler(httpd_req_t *req)
{
    char form[FORM_BUFFER_SIZE], text[PARAM_SIZE];
    struct tm utc;
    if (!read_form(req, form) || !form_value(form, "utc", text) || !parse_utc(text, &utc)) {
        return web_respond(req, HTTPD_400, "invalid date or time");
    }
    ntp_clock_reading_t reading;
    ntp_clock_read(esp_timer_get_time(), &reading);
    if (reading.source == NTP_CLOCK_GNSS || reading.source == NTP_CLOCK_NETWORK) {
        return web_respond(req, HTTP_STATUS_CONFLICT, "GNSS or network time is available, manual time is not accepted");
    }
    if (rtc_write(&utc) != ESP_OK) {
        return web_respond(req, HTTPD_500, "could not write the RTC");
    }
    ntp_clock_anchor_local(timegm(&utc), esp_timer_get_time());
    return web_respond(req, HTTPD_200, "time set");
}

static esp_err_t network_get_handler(httpd_req_t *req)
{
    static struct json_out out;
    network_settings_t network;
    if (settings_load_network(&network) != ESP_OK) {
        return web_respond(req, HTTPD_500, "could not read settings");
    }
    out.length = 0;
    json_printf(&out, "{\"eth_static\":%s,\"eth_ip\":\"%s\",\"eth_mask\":\"%s\",\"eth_gateway\":\"%s\","
                      "\"eth_dns\":\"%s\",\"wifi_enabled\":%s,\"wifi_password_set\":%s,\"wifi_static\":%s,"
                      "\"wifi_ip\":\"%s\",\"wifi_mask\":\"%s\",\"wifi_gateway\":\"%s\",\"wifi_dns\":\"%s\",\"wifi_ssid\":",
                network.eth_static ? "true" : "false", network.eth_ip, network.eth_mask, network.eth_gateway,
                network.eth_dns, network.wifi_enabled ? "true" : "false", network.wifi_password[0] ? "true" : "false",
                network.wifi_static ? "true" : "false", network.wifi_ip, network.wifi_mask, network.wifi_gateway,
                network.wifi_dns);
    json_string(&out, network.wifi_ssid);
    json_printf(&out, ",\"hostname\":\"%s\"}", network.hostname);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out.buffer, out.length);
}

// A key that is absent leaves the field unchanged; false means the value does not fit.
static bool assign_field(const char *form, const char *key, char *field, size_t size)
{
    char value[PARAM_SIZE];
    if (!form_value(form, key, value)) {
        return true;
    }
    if (strlen(value) >= size) {
        return false;
    }
    strcpy(field, value);
    return true;
}

static esp_err_t network_post_handler(httpd_req_t *req)
{
    char form[FORM_BUFFER_SIZE], value[PARAM_SIZE];
    if (!read_form(req, form)) {
        return web_respond(req, HTTPD_400, "empty or oversized request");
    }
    network_settings_t network;
    if (settings_load_network(&network) != ESP_OK) {
        return web_respond(req, HTTPD_500, "could not read settings");
    }
    const network_settings_t previous = network;

    bool fits = assign_field(form, "eth_ip", network.eth_ip, sizeof(network.eth_ip))
                && assign_field(form, "eth_mask", network.eth_mask, sizeof(network.eth_mask))
                && assign_field(form, "eth_gateway", network.eth_gateway, sizeof(network.eth_gateway))
                && assign_field(form, "eth_dns", network.eth_dns, sizeof(network.eth_dns))
                && assign_field(form, "wifi_ip", network.wifi_ip, sizeof(network.wifi_ip))
                && assign_field(form, "wifi_mask", network.wifi_mask, sizeof(network.wifi_mask))
                && assign_field(form, "wifi_gateway", network.wifi_gateway, sizeof(network.wifi_gateway))
                && assign_field(form, "wifi_dns", network.wifi_dns, sizeof(network.wifi_dns))
                && assign_field(form, "wifi_ssid", network.wifi_ssid, sizeof(network.wifi_ssid))
                && assign_field(form, "wifi_password", network.wifi_password, sizeof(network.wifi_password))
                && assign_field(form, "hostname", network.hostname, sizeof(network.hostname));
    if (!fits) {
        return web_respond(req, HTTPD_400, "a field is too long");
    }
    if (form_value(form, "eth_mode", value)) {
        network.eth_static = strcmp(value, "static") == 0;
    }
    if (form_value(form, "wifi_mode", value)) {
        network.wifi_static = strcmp(value, "static") == 0;
    }
    if (form_value(form, "wifi_enabled", value)) {
        network.wifi_enabled = strcmp(value, "1") == 0;
    }
    // An empty password with an unchanged SSID means "keep the stored one".
    if (network.wifi_password[0] == '\0' && strcmp(network.wifi_ssid, previous.wifi_ssid) == 0) {
        strcpy(network.wifi_password, previous.wifi_password);
    }

    const char *error = settings_network_error(&network);
    if (error) {
        return web_respond(req, HTTPD_400, error);
    }
    if (settings_save_network(&network) != ESP_OK) {
        return web_respond(req, HTTPD_500, "could not store settings");
    }
    return web_respond(req, HTTPD_200, "saved, restart to apply");
}

static esp_err_t reboot_handler(httpd_req_t *req)
{
    web_respond(req, HTTPD_200, "restarting");
    vTaskDelay(pdMS_TO_TICKS(REBOOT_DELAY_MS));
    esp_restart();
}

static esp_err_t sensors_get_handler(httpd_req_t *req)
{
    static struct json_out out;
    sensor_info_t sensors[SENSORS_MAX];
    const size_t count = sensors_snapshot(sensors);
    out.length = 0;
    json_printf(&out, "[");
    for (size_t i = 0; i < count; i++) {
        json_printf(&out, "%s{\"id\":\"%s\",\"name\":\"%s\",\"present\":%s,\"temperature\":", i ? "," : "",
                    sensors[i].id, sensors[i].name, sensors[i].present ? "true" : "false");
        if (sensors[i].has_reading) {
            json_printf(&out, "%.2f}", sensors[i].temperature_c);
        } else {
            json_printf(&out, "null}");
        }
    }
    json_printf(&out, "]");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, out.buffer, out.length);
}

static esp_err_t sensors_post_handler(httpd_req_t *req)
{
    char form[FORM_BUFFER_SIZE], id[PARAM_SIZE], name[PARAM_SIZE];
    if (!read_form(req, form) || !form_value(form, "id", id) || !form_value(form, "name", name)) {
        return web_respond(req, HTTPD_400, "missing id or name");
    }
    switch (sensors_rename(id, name)) {
    case ESP_OK:
        return web_respond(req, HTTPD_200, "name saved");
    case ESP_ERR_NOT_FOUND:
        return web_respond(req, HTTPD_400, "unknown sensor");
    case ESP_ERR_INVALID_STATE:
        return web_respond(req, HTTP_STATUS_CONFLICT, "name already used by another sensor");
    case ESP_ERR_INVALID_ARG:
        return web_respond(req, HTTPD_400, "name must be letters, digits or underscore, not starting with a digit");
    default:
        return web_respond(req, HTTPD_500, "could not store the name");
    }
}

static esp_err_t munin_get_handler(httpd_req_t *req)
{
    char plugin[SETTINGS_MUNIN_PLUGIN_MAX_LENGTH], body[96];
    munin_get_plugin_name(plugin);
    snprintf(body, sizeof(body), "{\"plugin\":\"%s\"}", plugin);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t munin_post_handler(httpd_req_t *req)
{
    char form[FORM_BUFFER_SIZE], plugin[PARAM_SIZE];
    if (!read_form(req, form) || !form_value(form, "plugin", plugin)) {
        return web_respond(req, HTTPD_400, "missing plugin name");
    }
    switch (munin_set_plugin_name(plugin)) {
    case ESP_OK:
        return web_respond(req, HTTPD_200, "plugin name saved");
    case ESP_ERR_INVALID_ARG:
        return web_respond(req, HTTPD_400, "name must be letters, digits or underscore, not starting with a digit");
    default:
        return web_respond(req, HTTPD_500, "could not store the name");
    }
}

static esp_err_t gnss_config_get_handler(httpd_req_t *req)
{
    gnss_receiver_t receiver;
    char body[192];
    gnss_config_get(&receiver);
    snprintf(body, sizeof(body),
             "{\"known\":%s,\"saved\":%s,\"delays_managed\":%s,\"dynamic_model\":%u,"
             "\"cable_delay_ns\":%d,\"user_delay_ns\":%ld}",
             receiver.known ? "true" : "false", receiver.saved ? "true" : "false",
             receiver.delays_managed ? "true" : "false", receiver.dynamic_model, receiver.cable_delay_ns,
             (long)receiver.user_delay_ns);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

static bool parse_integer(const char *text, long *value)
{
    char *end;
    *value = strtol(text, &end, 10);
    return end != text && *end == '\0';
}

static esp_err_t gnss_config_post_handler(httpd_req_t *req)
{
    char form[FORM_BUFFER_SIZE], text[PARAM_SIZE];
    long cable, user;
    if (!read_form(req, form) || !form_value(form, "cable_delay_ns", text) || !parse_integer(text, &cable)
        || !form_value(form, "user_delay_ns", text) || !parse_integer(text, &user)) {
        return web_respond(req, HTTPD_400, "cable_delay_ns and user_delay_ns must be integers");
    }
    if (cable < 0 || cable > GNSS_DELAY_CABLE_MAX_NS || user < -GNSS_DELAY_USER_LIMIT_NS
        || user > GNSS_DELAY_USER_LIMIT_NS) {
        return web_respond(req, HTTPD_400, "delay out of range (cable 0-32767 ns, user +-1000000 ns)");
    }
    bool written = false;
    switch (gnss_config_set_pps_delay((int16_t)cable, (int32_t)user, &written)) {
    case ESP_OK:
        return web_respond(req, HTTPD_200, written ? "delays written and saved in the receiver"
                                                   : "the receiver already has these values");
    case ESP_ERR_TIMEOUT:
        return web_respond(req, HTTPD_500, "the receiver did not answer");
    case ESP_FAIL:
        return web_respond(req, HTTPD_500, "the receiver refused the configuration");
    default:
        return web_respond(req, HTTPD_500, "could not apply the delays");
    }
}

static esp_err_t upstream_post_handler(httpd_req_t *req)
{
    char form[FORM_BUFFER_SIZE], server[PARAM_SIZE];
    if (!read_form(req, form) || !form_value(form, "server", server)) {
        return web_respond(req, HTTPD_400, "missing server");
    }
    switch (upstream_set_server(server)) {
    case ESP_OK:
        return web_respond(req, HTTPD_200, server[0] ? "server saved, syncing" : "network fallback disabled");
    case ESP_ERR_INVALID_ARG:
        return web_respond(req, HTTPD_400, "use a host name or IPv4 address (letters, digits, dot, dash)");
    default:
        return web_respond(req, HTTPD_500, "could not store the server");
    }
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html_start, HTTPD_RESP_USE_STRLEN);
}

esp_err_t web_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = HTTP_STACK_SIZE;
    config.max_uri_handlers = HTTP_MAX_URI_HANDLERS;
    config.max_open_sockets = HTTP_MAX_OPEN_SOCKETS;
    config.lru_purge_enable = true;

    httpd_handle_t server;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "httpd start");

    const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = index_handler},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/timezone", .method = HTTP_POST, .handler = timezone_handler},
        {.uri = "/api/time", .method = HTTP_POST, .handler = time_handler},
        {.uri = "/api/network", .method = HTTP_GET, .handler = network_get_handler},
        {.uri = "/api/network", .method = HTTP_POST, .handler = network_post_handler},
        {.uri = "/api/reboot", .method = HTTP_POST, .handler = reboot_handler},
        {.uri = "/api/ota", .method = HTTP_POST, .handler = ota_upload_handler},
        {.uri = "/api/sensors", .method = HTTP_GET, .handler = sensors_get_handler},
        {.uri = "/api/sensors", .method = HTTP_POST, .handler = sensors_post_handler},
        {.uri = "/api/gnss/config", .method = HTTP_GET, .handler = gnss_config_get_handler},
        {.uri = "/api/gnss/config", .method = HTTP_POST, .handler = gnss_config_post_handler},
        {.uri = "/api/upstream", .method = HTTP_POST, .handler = upstream_post_handler},
        {.uri = "/api/munin", .method = HTTP_GET, .handler = munin_get_handler},
        {.uri = "/api/munin", .method = HTTP_POST, .handler = munin_post_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &routes[i]), TAG, "register %s", routes[i].uri);
    }
    return ESP_OK;
}

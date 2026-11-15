#include "gpsd.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <time.h>

#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "gnss.h"
#include "ntp_clock.h"

#define GPSD_PORT 2947
#define MAX_CLIENTS 3
#define LISTEN_BACKLOG 2
#define COMMAND_SIZE 160
#define MESSAGE_SIZE 6144
#define SATELLITE_RESERVE 160
#define SEND_TIMEOUT_MS 200
#define REPORT_PERIOD_US 500000
#define GNSS_STALE_US 5000000
#define TASK_STACK_SIZE 6144
#define PROTOCOL_MAJOR 3
#define PROTOCOL_MINOR 14
#define DEVICE_PATH "gnss"
#define KMH_PER_MS 3.6f

#define GNSS_ID_GALILEO 2
#define GNSS_ID_BEIDOU 3
#define GNSS_ID_QZSS 5
#define GNSS_ID_GLONASS 6
#define PRN_OFFSET_GALILEO 300
#define PRN_OFFSET_BEIDOU 400
#define PRN_OFFSET_QZSS 192
#define PRN_OFFSET_GLONASS 64

#define MODE_UNKNOWN 0
#define MODE_NO_FIX 1
#define MODE_2D 2
#define MODE_3D 3

static const char *TAG = "gpsd";

typedef struct {
    int fd;
    int64_t connected_us;
    bool watching;
    char command[COMMAND_SIZE];
    size_t length;
    int depth;
} client_t;

static client_t clients[MAX_CLIENTS];
static char message[MESSAGE_SIZE];

static void append(size_t *length, const char *format, ...)
{
    va_list arguments;
    va_start(arguments, format);
    const int written = vsnprintf(message + *length, sizeof(message) - *length, format, arguments);
    va_end(arguments);
    if (written > 0) {
        *length = MIN(*length + (size_t)written, sizeof(message) - 1);
    }
}

static bool client_active(const client_t *client)
{
    return client->fd > 0;
}

static void drop_client(client_t *client)
{
    close(client->fd);
    client->fd = 0;
}

static bool send_message(client_t *client, size_t length)
{
    const char *data = message;
    while (length) {
        const int sent = send(client->fd, data, length, 0);
        if (sent <= 0) {
            drop_client(client);
            return false;
        }
        data += sent;
        length -= (size_t)sent;
    }
    return true;
}

static int fix_mode(const gnss_status_t *status)
{
    if (!status->updated_us || esp_timer_get_time() - status->updated_us > GNSS_STALE_US) {
        return MODE_UNKNOWN;
    }
    return status->fix ? MAX(status->fix_mode, MODE_2D) : MODE_NO_FIX;
}

static size_t build_version(void)
{
    size_t length = 0;
    append(&length, "{\"class\":\"VERSION\",\"release\":\"%s\",\"rev\":\"%s\",\"proto_major\":%d,\"proto_minor\":%d}\r\n",
           esp_app_get_description()->version, esp_app_get_description()->project_name, PROTOCOL_MAJOR, PROTOCOL_MINOR);
    return length;
}

static size_t build_devices(void)
{
    size_t length = 0;
    append(&length,
           "{\"class\":\"DEVICES\",\"devices\":[{\"class\":\"DEVICE\",\"path\":\"" DEVICE_PATH "\","
           "\"driver\":\"u-blox\",\"bps\":%d,\"parity\":\"N\",\"stopbits\":1}]}\r\n",
           GNSS_BAUD);
    return length;
}

static size_t build_watch(bool enabled)
{
    size_t length = 0;
    append(&length,
           "{\"class\":\"WATCH\",\"enable\":%s,\"json\":true,\"nmea\":false,\"raw\":0,\"scaled\":false,"
           "\"timing\":false,\"split24\":false,\"pps\":false}\r\n",
           enabled ? "true" : "false");
    return length;
}

static size_t build_error(const char *text)
{
    size_t length = 0;
    append(&length, "{\"class\":\"ERROR\",\"message\":\"%s\"}\r\n", text);
    return length;
}

// The time is reported only while it comes from the receiver: the RTC or the network would pass for a GNSS fix.
static void append_time(size_t *length)
{
    ntp_clock_reading_t reading;
    ntp_clock_read(esp_timer_get_time(), &reading);
    if (reading.source != NTP_CLOCK_GNSS) {
        return;
    }
    const time_t seconds = reading.unix_us / 1000000;
    struct tm utc;
    gmtime_r(&seconds, &utc);
    append(length, ",\"time\":\"%04d-%02d-%02dT%02d:%02d:%02d.%03dZ\"", utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
           utc.tm_hour, utc.tm_min, utc.tm_sec, (int)(reading.unix_us % 1000000 / 1000));
}

static size_t build_tpv(void)
{
    gnss_status_t status;
    gnss_get_status(&status);
    const int mode = fix_mode(&status);
    size_t length = 0;
    append(&length, "{\"class\":\"TPV\",\"device\":\"" DEVICE_PATH "\",\"mode\":%d", mode);
    if (mode >= MODE_2D) {
        append_time(&length);
        if (status.has_position) {
            append(&length, ",\"lat\":%.9f,\"lon\":%.9f", status.latitude_deg, status.longitude_deg);
            if (mode == MODE_3D) {
                append(&length, ",\"altMSL\":%.1f,\"alt\":%.1f", (double)status.altitude_m, (double)status.altitude_m);
            }
        }
        append(&length, ",\"speed\":%.3f,\"track\":%.1f", (double)(status.speed_kmh / KMH_PER_MS),
               (double)status.course_deg);
    }
    append(&length, "}\r\n");
    return length;
}

// gpsd numbers satellites of all constellations in one space; the same id would otherwise repeat across them.
static unsigned prn(const gnss_satellite_t *satellite)
{
    switch (satellite->gnss_id) {
    case GNSS_ID_GALILEO:
        return PRN_OFFSET_GALILEO + satellite->id;
    case GNSS_ID_BEIDOU:
        return PRN_OFFSET_BEIDOU + satellite->id;
    case GNSS_ID_QZSS:
        return PRN_OFFSET_QZSS + satellite->id;
    case GNSS_ID_GLONASS:
        return PRN_OFFSET_GLONASS + satellite->id;
    default:
        return satellite->id;
    }
}

static size_t build_sky(void)
{
    gnss_status_t status;
    gnss_get_status(&status);
    gnss_satellite_t satellites[GNSS_MAX_SATELLITES];
    int64_t age_us;
    size_t count = gnss_get_satellites(satellites, &age_us);
    if (age_us > GNSS_STALE_US) {
        count = 0;
    }

    size_t length = 0;
    append(&length, "{\"class\":\"SKY\",\"device\":\"" DEVICE_PATH "\"");
    if (fix_mode(&status) >= MODE_2D) {
        append(&length, ",\"hdop\":%.2f,\"pdop\":%.2f", (double)status.hdop, (double)status.pdop);
    }
    append(&length, ",\"satellites\":[");
    for (size_t i = 0; i < count && sizeof(message) - length > SATELLITE_RESERVE; i++) {
        const gnss_satellite_t *satellite = &satellites[i];
        append(&length, "%s{\"PRN\":%u,\"gnssid\":%u,\"svid\":%u", i ? "," : "", prn(satellite), satellite->gnss_id,
               satellite->id);
        if (satellite->elevation_deg != GNSS_UNKNOWN && satellite->azimuth_deg != GNSS_UNKNOWN) {
            append(&length, ",\"el\":%d,\"az\":%d", satellite->elevation_deg, satellite->azimuth_deg);
        }
        if (satellite->snr_dbhz != GNSS_UNKNOWN) {
            append(&length, ",\"ss\":%d", satellite->snr_dbhz);
        }
        append(&length, ",\"used\":%s}", satellite->used ? "true" : "false");
    }
    append(&length, "]}\r\n");
    return length;
}

static void broadcast(size_t length)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_active(&clients[i]) && clients[i].watching) {
            send_message(&clients[i], length);
        }
    }
}

// TPV and SKY go out half a period apart: gps3, which Home Assistant uses, discards the rest of its read buffer after
// the first line, so a SKY that arrives together with a TPV is lost.
static void report_to_watchers(void)
{
    static bool sky_next;
    broadcast(sky_next ? build_sky() : build_tpv());
    sky_next = !sky_next;
}

static void remove_spaces(char *text)
{
    char *target = text;
    for (const char *source = text; *source; source++) {
        if (*source != ' ' && *source != '\t' && *source != '\r' && *source != '\n') {
            *target++ = *source;
        }
    }
    *target = '\0';
}

static bool starts_with(const char *text, const char *prefix)
{
    return strncmp(text, prefix, strlen(prefix)) == 0;
}

static void handle_command(client_t *client)
{
    client->command[client->length] = '\0';
    remove_spaces(client->command);
    const char *command = client->command;
    if (!*command) {
        return;
    }
    if (starts_with(command, "?WATCH")) {
        if (strchr(command, '{')) {
            client->watching = !strstr(command, "\"enable\":false");
        }
        if (send_message(client, build_devices())) {
            send_message(client, build_watch(client->watching));
        }
    } else if (starts_with(command, "?VERSION")) {
        send_message(client, build_version());
    } else if (starts_with(command, "?DEVICES")) {
        send_message(client, build_devices());
    } else {
        send_message(client, build_error("Unsupported request"));
    }
}

static void read_client(client_t *client)
{
    char chunk[64];
    const int received = recv(client->fd, chunk, sizeof(chunk), 0);
    if (received <= 0) {
        drop_client(client);
        return;
    }
    for (int i = 0; i < received && client_active(client); i++) {
        const char byte = chunk[i];
        if (byte == '{') {
            client->depth++;
        }
        const bool closes_object = byte == '}' && client->depth > 0 && --client->depth == 0;
        const bool ends_command = client->depth == 0 && (byte == '\n' || byte == ';');
        if (!ends_command && client->length < sizeof(client->command) - 1) {
            client->command[client->length++] = byte;
        }
        if (closes_object || ends_command) {
            handle_command(client);
            client->length = 0;
        }
    }
}

static client_t *free_slot(void)
{
    client_t *oldest = &clients[0];
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!client_active(&clients[i])) {
            return &clients[i];
        }
        if (clients[i].connected_us < oldest->connected_us) {
            oldest = &clients[i];
        }
    }
    // A client that vanished without closing would otherwise lock everyone else out until its connection times out.
    drop_client(oldest);
    return oldest;
}

static void accept_client(int listener)
{
    const int fd = accept(listener, NULL, NULL);
    if (fd < 0) {
        return;
    }
    const struct timeval send_timeout = {.tv_usec = SEND_TIMEOUT_MS * 1000};
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
    client_t *client = free_slot();
    *client = (client_t){.fd = fd, .connected_us = esp_timer_get_time()};
    send_message(client, build_version());
}

static void server_task(void *arg)
{
    const int listener = (int)(intptr_t)arg;
    int64_t next_report_us = esp_timer_get_time() + REPORT_PERIOD_US;
    while (true) {
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(listener, &readable);
        int highest = listener;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (client_active(&clients[i])) {
                FD_SET(clients[i].fd, &readable);
                highest = MAX(highest, clients[i].fd);
            }
        }
        const int64_t wait_us = MAX(next_report_us - esp_timer_get_time(), 0);
        struct timeval timeout = {.tv_sec = wait_us / 1000000, .tv_usec = wait_us % 1000000};
        if (select(highest + 1, &readable, NULL, NULL, &timeout) < 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (client_active(&clients[i]) && FD_ISSET(clients[i].fd, &readable)) {
                read_client(&clients[i]);
            }
        }
        if (FD_ISSET(listener, &readable)) {
            accept_client(listener);
        }
        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_report_us) {
            report_to_watchers();
            next_report_us = MAX(next_report_us + REPORT_PERIOD_US, now_us);
        }
    }
}

esp_err_t gpsd_start(void)
{
    const int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    ESP_RETURN_ON_FALSE(listener >= 0, ESP_FAIL, TAG, "socket: errno %d", errno);
    const struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(GPSD_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listener, (const struct sockaddr *)&address, sizeof(address)) < 0 || listen(listener, LISTEN_BACKLOG) < 0) {
        ESP_LOGE(TAG, "bind/listen: errno %d", errno);
        close(listener);
        return ESP_FAIL;
    }
    ESP_RETURN_ON_FALSE(xTaskCreate(server_task, "gpsd", TASK_STACK_SIZE, (void *)(intptr_t)listener, 3, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    ESP_LOGI(TAG, "listening on tcp/%d", GPSD_PORT);
    return ESP_OK;
}

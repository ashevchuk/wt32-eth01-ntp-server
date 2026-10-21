#include "upstream.h"

#include <ctype.h>
#include <string.h>
#include <time.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#include "ntp_clock.h"
#include "rtc.h"

#define NTP_PORT "123"
#define PACKET_SIZE 48
#define NTP_UNIX_EPOCH_OFFSET_S 2208988800LL
#define NTP_ERA_SECONDS (1LL << 32)
#define MICROS_PER_SECOND 1000000LL

#define SAMPLES 4
#define SAMPLE_SPACING_MS 500
#define REPLY_TIMEOUT_MS 1500
#define MAX_ACCEPTED_DELAY_US 500000
#define SYNC_INTERVAL_MS (15 * 60 * 1000)
#define RETRY_INTERVAL_MS 10000
#define MAX_STRATUM 15
#define RTC_WRITE_INTERVAL_US (3600 * MICROS_PER_SECOND)
#define COARSE_WAIT_THRESHOLD_US 30000
#define COARSE_WAIT_MARGIN_US 20000
#define FINE_WAIT_LIMIT_US 50000
#define TASK_STACK_SIZE 4096
#define TASK_PRIORITY 3

#define CLIENT_FLAGS ((4 << 3) | 3)
#define MODE_SERVER 4
#define LI_UNSYNCHRONIZED 3

enum field_offset {
    OFFSET_FLAGS = 0,
    OFFSET_STRATUM = 1,
    OFFSET_ROOT_DELAY = 4,
    OFFSET_ORIGIN_TIME = 24,
    OFFSET_RECEIVE_TIME = 32,
    OFFSET_TRANSMIT_TIME = 40,
};

struct sample {
    int64_t delay_us;
    int64_t server_unix_us;
    int64_t received_mono_us;
    int64_t server_root_delay_us;
    uint8_t stratum;
};

static const char *TAG = "upstream";

static upstream_status_t status;
static int64_t last_sync_mono_us;
static int64_t last_rtc_write_us;
static portMUX_TYPE status_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t task_handle;

static uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = value >> 24;
    p[1] = value >> 16;
    p[2] = value >> 8;
    p[3] = value;
}

static int64_t ntp_timestamp_to_unix_us(const uint8_t *p)
{
    const int64_t seconds = get_u32(p);
    // Values below the 1970 offset belong to NTP era 1, which starts in 2036.
    const int64_t unix_seconds = seconds >= NTP_UNIX_EPOCH_OFFSET_S ? seconds - NTP_UNIX_EPOCH_OFFSET_S
                                                                    : seconds + NTP_ERA_SECONDS - NTP_UNIX_EPOCH_OFFSET_S;
    return unix_seconds * MICROS_PER_SECOND + (int64_t)(((uint64_t)get_u32(p + 4) * MICROS_PER_SECOND) >> 32);
}

static bool server_name_valid(const char *name)
{
    const size_t length = strlen(name);
    if (length >= SETTINGS_UPSTREAM_MAX_LENGTH) {
        return false;
    }
    if (length > 0 && (name[0] == '.' || name[0] == '-')) {
        return false;
    }
    for (size_t i = 0; i < length; i++) {
        if (!isalnum((unsigned char)name[i]) && name[i] != '.' && name[i] != '-') {
            return false;
        }
    }
    return true;
}

// One request/response round trip; the reply is accepted only if it answers this very request.
static esp_err_t query(int sock, const struct sockaddr_in *address, struct sample *sample)
{
    uint8_t request[PACKET_SIZE] = {0};
    request[OFFSET_FLAGS] = CLIENT_FLAGS;
    const uint32_t token_high = esp_random(), token_low = esp_random();
    put_u32(request + OFFSET_TRANSMIT_TIME, token_high);
    put_u32(request + OFFSET_TRANSMIT_TIME + 4, token_low);

    const int64_t sent_us = esp_timer_get_time();
    if (sendto(sock, request, sizeof(request), 0, (const struct sockaddr *)address, sizeof(*address)) < 0) {
        return ESP_FAIL;
    }
    uint8_t reply[PACKET_SIZE];
    const ssize_t length = recv(sock, reply, sizeof(reply), 0);
    const int64_t received_us = esp_timer_get_time();
    if (length < PACKET_SIZE) {
        return ESP_ERR_TIMEOUT;
    }
    const uint8_t stratum = reply[OFFSET_STRATUM];
    const bool plausible = (reply[OFFSET_FLAGS] & 7) == MODE_SERVER && (reply[OFFSET_FLAGS] >> 6) != LI_UNSYNCHRONIZED
                           && stratum >= 1 && stratum <= MAX_STRATUM
                           && get_u32(reply + OFFSET_ORIGIN_TIME) == token_high
                           && get_u32(reply + OFFSET_ORIGIN_TIME + 4) == token_low
                           && get_u32(reply + OFFSET_TRANSMIT_TIME) != 0;
    if (!plausible) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const int64_t server_received_us = ntp_timestamp_to_unix_us(reply + OFFSET_RECEIVE_TIME);
    const int64_t server_sent_us = ntp_timestamp_to_unix_us(reply + OFFSET_TRANSMIT_TIME);
    const int64_t delay_us = (received_us - sent_us) - (server_sent_us - server_received_us);
    if (delay_us < 0 || delay_us > MAX_ACCEPTED_DELAY_US) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    *sample = (struct sample){
        .delay_us = delay_us,
        .server_unix_us = server_sent_us + delay_us / 2,
        .received_mono_us = received_us,
        .server_root_delay_us = (int64_t)((uint64_t)get_u32(reply + OFFSET_ROOT_DELAY) * MICROS_PER_SECOND >> 16),
        .stratum = stratum,
    };
    return ESP_OK;
}

// The DS3231 only takes whole seconds, so the write is timed to land on a second boundary of our own clock.
static void write_rtc_on_next_second(void)
{
    ntp_clock_reading_t now;
    ntp_clock_read(esp_timer_get_time(), &now);
    const int64_t target_second = now.unix_us / MICROS_PER_SECOND + 1;
    const int64_t remaining_us = target_second * MICROS_PER_SECOND - now.unix_us;
    if (remaining_us > COARSE_WAIT_THRESHOLD_US) {
        vTaskDelay(pdMS_TO_TICKS((remaining_us - COARSE_WAIT_MARGIN_US) / 1000));
    }
    ntp_clock_read(esp_timer_get_time(), &now);
    const int64_t left_us = target_second * MICROS_PER_SECOND - now.unix_us;
    if (left_us < 0 || left_us > FINE_WAIT_LIMIT_US) {
        return;
    }
    esp_rom_delay_us((uint32_t)left_us);

    const time_t seconds = (time_t)target_second;
    struct tm utc;
    gmtime_r(&seconds, &utc);
    if (rtc_write(&utc) == ESP_OK) {
        last_rtc_write_us = esp_timer_get_time();
        ESP_LOGI(TAG, "rtc set from the network");
    }
}

static void apply_sample(const struct sample *best, const struct sockaddr_in *address)
{
    ntp_clock_reading_t before;
    ntp_clock_read(best->received_mono_us, &before);
    const bool gnss_locked = before.source == NTP_CLOCK_GNSS;

    uint8_t reference_id[4];
    memcpy(reference_id, &address->sin_addr.s_addr, sizeof(reference_id));
    const uint8_t stratum = best->stratum < MAX_STRATUM ? best->stratum + 1 : MAX_STRATUM;
    ntp_clock_anchor_network(best->server_unix_us, best->received_mono_us, stratum, reference_id,
                             best->server_root_delay_us + best->delay_us);

    portENTER_CRITICAL(&status_lock);
    status.synced = true;
    status.server_stratum = best->stratum;
    status.delay_us = (int32_t)best->delay_us;
    status.has_offset = gnss_locked;
    status.offset_us = gnss_locked ? before.unix_us - best->server_unix_us : 0;
    last_sync_mono_us = best->received_mono_us;
    portEXIT_CRITICAL(&status_lock);

    const bool rtc_due = last_rtc_write_us == 0 || esp_timer_get_time() - last_rtc_write_us >= RTC_WRITE_INTERVAL_US;
    if (!gnss_locked && rtc_due) {
        write_rtc_on_next_second();
    }
}

static esp_err_t sync_once(const char *server)
{
    const struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_DGRAM};
    struct addrinfo *found = NULL;
    if (getaddrinfo(server, NTP_PORT, &hints, &found) != 0 || !found) {
        return ESP_ERR_NOT_FOUND;
    }
    const struct sockaddr_in address = *(const struct sockaddr_in *)found->ai_addr;
    freeaddrinfo(found);

    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    ESP_RETURN_ON_FALSE(sock >= 0, ESP_FAIL, TAG, "socket: errno %d", errno);
    const struct timeval timeout = {.tv_sec = REPLY_TIMEOUT_MS / 1000, .tv_usec = (REPLY_TIMEOUT_MS % 1000) * 1000};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    struct sample best = {.delay_us = INT64_MAX};
    bool have_sample = false;
    for (int i = 0; i < SAMPLES; i++) {
        struct sample candidate;
        if (query(sock, &address, &candidate) == ESP_OK && candidate.delay_us < best.delay_us) {
            best = candidate;
            have_sample = true;
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_SPACING_MS));
    }
    close(sock);
    if (!have_sample) {
        return ESP_ERR_TIMEOUT;
    }
    apply_sample(&best, &address);
    ESP_LOGI(TAG, "%s: stratum %u, delay %lld us", server, best.stratum, (long long)best.delay_us);
    return ESP_OK;
}

static void upstream_task(void *arg)
{
    while (true) {
        char server[SETTINGS_UPSTREAM_MAX_LENGTH];
        portENTER_CRITICAL(&status_lock);
        memcpy(server, status.server, sizeof(server));
        portEXIT_CRITICAL(&status_lock);

        uint32_t wait_ms = portMAX_DELAY;
        if (server[0]) {
            const esp_err_t err = sync_once(server);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "%s: %s", server, esp_err_to_name(err));
            }
            wait_ms = err == ESP_OK ? SYNC_INTERVAL_MS : RETRY_INTERVAL_MS;
        }
        // A new server name wakes the task early.
        ulTaskNotifyTake(pdTRUE, wait_ms == portMAX_DELAY ? portMAX_DELAY : pdMS_TO_TICKS(wait_ms));
    }
}

esp_err_t upstream_set_server(const char *server)
{
    ESP_RETURN_ON_FALSE(server_name_valid(server), ESP_ERR_INVALID_ARG, TAG, "invalid server");
    ESP_RETURN_ON_ERROR(settings_save_upstream(server), TAG, "store server");
    portENTER_CRITICAL(&status_lock);
    snprintf(status.server, sizeof(status.server), "%s", server);
    status.enabled = server[0] != '\0';
    status.synced = false;
    portEXIT_CRITICAL(&status_lock);
    xTaskNotifyGive(task_handle);
    return ESP_OK;
}

void upstream_get_status(upstream_status_t *out)
{
    portENTER_CRITICAL(&status_lock);
    *out = status;
    out->age_s = status.synced ? (esp_timer_get_time() - last_sync_mono_us) / MICROS_PER_SECOND : -1;
    portEXIT_CRITICAL(&status_lock);
}

esp_err_t upstream_start(void)
{
    char server[SETTINGS_UPSTREAM_MAX_LENGTH];
    ESP_RETURN_ON_ERROR(settings_load_upstream(server), TAG, "load server");
    snprintf(status.server, sizeof(status.server), "%s", server);
    status.enabled = server[0] != '\0';
    ESP_RETURN_ON_FALSE(xTaskCreate(upstream_task, "upstream", TASK_STACK_SIZE, NULL, TASK_PRIORITY, &task_handle) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

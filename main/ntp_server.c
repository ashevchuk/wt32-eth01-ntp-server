#include "ntp_server.h"

#include <stdint.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "leap.h"
#include "ntp_clock.h"

#define NTP_PORT 123
#define PACKET_SIZE 48
#define NTP_UNIX_EPOCH_OFFSET_S 2208988800ULL
#define MICROS_PER_SECOND 1000000ULL
#define INTERFACE_COUNT 2
#define RECV_TIMEOUT_S 1

#define MODE_CLIENT 3
#define MODE_SERVER 4
#define MIN_VERSION 1
#define MAX_VERSION 4
#define LI_UNSYNCHRONIZED 3

// log2 of the clock precision in seconds: 2^-10 s is about 1 ms, bounded by task wake-up jitter.
#define PRECISION_LOG2 (-10)
#define NTP_SHORT_FROM_US(us) ((uint32_t)((us) * 65536 / 1000000))

enum field_offset {
    OFFSET_FLAGS = 0,
    OFFSET_STRATUM = 1,
    OFFSET_POLL = 2,
    OFFSET_PRECISION = 3,
    OFFSET_ROOT_DELAY = 4,
    OFFSET_ROOT_DISPERSION = 8,
    OFFSET_REFERENCE_ID = 12,
    OFFSET_REFERENCE_TIME = 16,
    OFFSET_ORIGIN_TIME = 24,
    OFFSET_RECEIVE_TIME = 32,
    OFFSET_TRANSMIT_TIME = 40,
};

static const char *TAG = "ntp_server";

static volatile uint32_t request_count;

uint32_t ntp_server_request_count(void)
{
    return request_count;
}

static void put_u32(uint8_t *out, uint32_t value)
{
    out[0] = value >> 24;
    out[1] = value >> 16;
    out[2] = value >> 8;
    out[3] = value;
}

static void put_timestamp(uint8_t *out, int64_t unix_us)
{
    const uint64_t seconds = (uint64_t)(unix_us / (int64_t)MICROS_PER_SECOND) + NTP_UNIX_EPOCH_OFFSET_S;
    const uint64_t micros = (uint64_t)(unix_us % (int64_t)MICROS_PER_SECOND);
    put_u32(out, (uint32_t)seconds);
    put_u32(out + 4, (uint32_t)((micros << 32) / MICROS_PER_SECOND));
}

static void build_reply(uint8_t reply[PACKET_SIZE], const uint8_t request[PACKET_SIZE], int64_t receive_mono_us)
{
    ntp_clock_reading_t received;
    ntp_clock_read(receive_mono_us, &received);

    const uint8_t version = (request[OFFSET_FLAGS] >> 3) & 7;
    memset(reply, 0, PACKET_SIZE);
    memcpy(reply + OFFSET_ORIGIN_TIME, request + OFFSET_TRANSMIT_TIME, 8);
    reply[OFFSET_POLL] = request[OFFSET_POLL];
    reply[OFFSET_PRECISION] = (uint8_t)PRECISION_LOG2;

    if (received.source == NTP_CLOCK_UNSYNCHRONIZED) {
        reply[OFFSET_FLAGS] = (LI_UNSYNCHRONIZED << 6) | (version << 3) | MODE_SERVER;
        reply[OFFSET_STRATUM] = received.stratum;
        return;
    }

    const uint8_t leap = leap_warning(receive_mono_us, received.unix_us / (int64_t)MICROS_PER_SECOND);
    reply[OFFSET_FLAGS] = (leap << 6) | (version << 3) | MODE_SERVER;
    reply[OFFSET_STRATUM] = received.stratum;
    put_u32(reply + OFFSET_ROOT_DELAY, NTP_SHORT_FROM_US(received.root_delay_us));
    put_u32(reply + OFFSET_ROOT_DISPERSION, NTP_SHORT_FROM_US(received.root_dispersion_us));
    memcpy(reply + OFFSET_REFERENCE_ID, received.reference_id, sizeof(received.reference_id));
    put_timestamp(reply + OFFSET_REFERENCE_TIME, received.reference_unix_us);
    put_timestamp(reply + OFFSET_RECEIVE_TIME, received.unix_us);

    ntp_clock_reading_t transmitted;
    ntp_clock_read(esp_timer_get_time(), &transmitted);
    put_timestamp(reply + OFFSET_TRANSMIT_TIME, transmitted.unix_us);
}

static bool is_client_request(const uint8_t *packet, ssize_t length)
{
    const uint8_t version = (packet[OFFSET_FLAGS] >> 3) & 7;
    return length >= PACKET_SIZE && (packet[OFFSET_FLAGS] & 7) == MODE_CLIENT
           && version >= MIN_VERSION && version <= MAX_VERSION;
}

// One socket per interface, bound to that interface's address: a reply then leaves through the interface the request
// arrived on and carries its address as the source. A socket on INADDR_ANY answers from whichever interface routing
// finds first, and a client drops a reply that does not come from the address it asked.
struct listener {
    int sock;
    uint32_t address;
};

static const char *const INTERFACE_KEYS[INTERFACE_COUNT] = {"ETH_DEF", "WIFI_STA_DEF"};
static struct listener listeners[INTERFACE_COUNT] = {{-1, 0}, {-1, 0}};

static uint32_t interface_address(const char *key)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey(key);
    esp_netif_ip_info_t info;
    return netif && esp_netif_get_ip_info(netif, &info) == ESP_OK ? info.ip.addr : 0;
}

// Follows the interface's address; a failed bind is not retried until the address changes.
static void sync_listener(struct listener *listener, const char *key)
{
    const uint32_t address = interface_address(key);
    if (address == listener->address) {
        return;
    }
    if (listener->sock >= 0) {
        close(listener->sock);
        listener->sock = -1;
    }
    listener->address = address;
    if (address == 0) {
        return;
    }
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        return;
    }
    const struct sockaddr_in local = {
        .sin_family = AF_INET,
        .sin_port = htons(NTP_PORT),
        .sin_addr.s_addr = address,
    };
    if (bind(sock, (const struct sockaddr *)&local, sizeof(local)) < 0) {
        ESP_LOGE(TAG, "bind %s: errno %d", key, errno);
        close(sock);
        return;
    }
    const struct timeval timeout = {.tv_sec = RECV_TIMEOUT_S};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    listener->sock = sock;
}

static void answer_request(int sock)
{
    uint8_t request[PACKET_SIZE];
    struct sockaddr_in client;
    socklen_t client_length = sizeof(client);
    const ssize_t length = recvfrom(sock, request, sizeof(request), 0, (struct sockaddr *)&client, &client_length);
    const int64_t receive_mono_us = esp_timer_get_time();
    if (length < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGW(TAG, "recvfrom failed: errno %d", errno);
        }
        return;
    }
    if (!is_client_request(request, length)) {
        return;
    }
    uint8_t reply[PACKET_SIZE];
    build_reply(reply, request, receive_mono_us);
    request_count++;
    if (sendto(sock, reply, sizeof(reply), 0, (struct sockaddr *)&client, client_length) < 0) {
        ESP_LOGW(TAG, "sendto failed: errno %d", errno);
    }
}

static void server_task(void *arg)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_add(NULL));
    while (true) {
        esp_task_wdt_reset();
        fd_set readable;
        FD_ZERO(&readable);
        int highest = -1;
        for (int i = 0; i < INTERFACE_COUNT; i++) {
            sync_listener(&listeners[i], INTERFACE_KEYS[i]);
            if (listeners[i].sock >= 0) {
                FD_SET(listeners[i].sock, &readable);
                highest = listeners[i].sock > highest ? listeners[i].sock : highest;
            }
        }
        struct timeval timeout = {.tv_sec = RECV_TIMEOUT_S};
        if (select(highest + 1, &readable, NULL, NULL, &timeout) <= 0) {
            continue;
        }
        for (int i = 0; i < INTERFACE_COUNT; i++) {
            if (listeners[i].sock >= 0 && FD_ISSET(listeners[i].sock, &readable)) {
                answer_request(listeners[i].sock);
            }
        }
    }
}

esp_err_t ntp_server_start(void)
{
    ESP_RETURN_ON_FALSE(xTaskCreate(server_task, "ntp_server", 4096, NULL, 6, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

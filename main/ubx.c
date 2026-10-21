#include "ubx.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define SYNC_1 0xB5
#define SYNC_2 0x62
#define HEADER_SIZE 6
#define CHECKSUM_SIZE 2
#define CLASS_ACK 0x05
#define ID_ACK_NAK 0x00
#define ID_ACK_ACK 0x01
#define RX_QUEUE_LENGTH 4
#define REPLY_TIMEOUT_MS 1000
#define FRAME_GAP_TIMEOUT_US 100000
// NAV-SAT with 64 satellites is 776 bytes.
#define RX_MAX_PAYLOAD 800

struct frame {
    uint8_t cls;
    uint8_t id;
    uint16_t length;
    uint8_t payload[UBX_MAX_PAYLOAD];
};

static const char *TAG = "ubx";

static uart_port_t uart_port;
static QueueHandle_t rx_queue;
static SemaphoreHandle_t transaction_lock;

static uint8_t rx_buffer[HEADER_SIZE + RX_MAX_PAYLOAD + CHECKSUM_SIZE];
static ubx_handler_t handler;
static size_t rx_position;
static size_t rx_expected;
static int64_t rx_last_byte_us;

static void checksum(const uint8_t *data, size_t length, uint8_t *a, uint8_t *b)
{
    *a = 0;
    *b = 0;
    for (size_t i = 0; i < length; i++) {
        *a += data[i];
        *b += *a;
    }
}

esp_err_t ubx_init(uart_port_t port)
{
    uart_port = port;
    rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(struct frame));
    transaction_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(rx_queue && transaction_lock, ESP_ERR_NO_MEM, TAG, "allocation");
    return ESP_OK;
}

void ubx_set_handler(ubx_handler_t new_handler)
{
    handler = new_handler;
}

static void deliver_frame(void)
{
    uint8_t a, b;
    const size_t body = rx_expected - CHECKSUM_SIZE;
    checksum(rx_buffer + 2, body - 2, &a, &b);
    if (a != rx_buffer[body] || b != rx_buffer[body + 1]) {
        return;
    }
    struct frame frame = {
        .cls = rx_buffer[2],
        .id = rx_buffer[3],
        .length = (uint16_t)(rx_buffer[4] | (rx_buffer[5] << 8)),
    };
    if (handler) {
        handler(frame.cls, frame.id, rx_buffer + HEADER_SIZE, frame.length);
    }
    if (frame.length <= UBX_MAX_PAYLOAD) {
        memcpy(frame.payload, rx_buffer + HEADER_SIZE, frame.length);
        xQueueSend(rx_queue, &frame, 0);
    }
}

bool ubx_feed(uint8_t byte)
{
    const int64_t now_us = esp_timer_get_time();
    if (rx_position > 0 && now_us - rx_last_byte_us > FRAME_GAP_TIMEOUT_US) {
        rx_position = 0;
    }
    rx_last_byte_us = now_us;

    if (rx_position == 0) {
        if (byte != SYNC_1) {
            return false;
        }
        rx_buffer[rx_position++] = byte;
        return true;
    }
    if (rx_position == 1 && byte != SYNC_2) {
        rx_position = 0;
        return false;
    }

    rx_buffer[rx_position++] = byte;
    if (rx_position == HEADER_SIZE) {
        const size_t length = (size_t)(rx_buffer[4] | (rx_buffer[5] << 8));
        if (length > RX_MAX_PAYLOAD) {
            rx_position = 0;
            return true;
        }
        rx_expected = HEADER_SIZE + length + CHECKSUM_SIZE;
    }
    if (rx_position >= HEADER_SIZE && rx_position == rx_expected) {
        deliver_frame();
        rx_position = 0;
    }
    return true;
}

static esp_err_t send_message(uint8_t cls, uint8_t id, const uint8_t *payload, size_t length)
{
    ESP_RETURN_ON_FALSE(length <= UBX_MAX_PAYLOAD, ESP_ERR_INVALID_SIZE, TAG, "payload too long");
    uint8_t message[HEADER_SIZE + UBX_MAX_PAYLOAD + CHECKSUM_SIZE];
    message[0] = SYNC_1;
    message[1] = SYNC_2;
    message[2] = cls;
    message[3] = id;
    message[4] = (uint8_t)length;
    message[5] = (uint8_t)(length >> 8);
    if (length) {
        memcpy(message + HEADER_SIZE, payload, length);
    }
    checksum(message + 2, 4 + length, &message[HEADER_SIZE + length], &message[HEADER_SIZE + length + 1]);
    const size_t total = HEADER_SIZE + length + CHECKSUM_SIZE;
    ESP_RETURN_ON_FALSE(uart_write_bytes(uart_port, message, total) == (int)total, ESP_FAIL, TAG, "uart write");
    return uart_wait_tx_done(uart_port, pdMS_TO_TICKS(REPLY_TIMEOUT_MS));
}

esp_err_t ubx_poll(uint8_t cls, uint8_t id, const uint8_t *payload, size_t length, uint8_t *reply,
                   size_t reply_size, size_t *reply_length)
{
    xSemaphoreTake(transaction_lock, portMAX_DELAY);
    xQueueReset(rx_queue);
    esp_err_t result = send_message(cls, id, payload, length);
    const int64_t deadline_us = esp_timer_get_time() + REPLY_TIMEOUT_MS * 1000LL;
    while (result == ESP_OK) {
        const int64_t left_us = deadline_us - esp_timer_get_time();
        struct frame frame;
        if (left_us <= 0 || xQueueReceive(rx_queue, &frame, pdMS_TO_TICKS(left_us / 1000 + 1)) != pdTRUE) {
            result = ESP_ERR_TIMEOUT;
        } else if (frame.cls == cls && frame.id == id) {
            if (frame.length > reply_size) {
                result = ESP_ERR_INVALID_SIZE;
            } else {
                memcpy(reply, frame.payload, frame.length);
                *reply_length = frame.length;
            }
            break;
        } else if (frame.cls == CLASS_ACK && frame.id == ID_ACK_NAK && frame.length >= 2
                   && frame.payload[0] == cls && frame.payload[1] == id) {
            result = ESP_FAIL;
        }
    }
    xSemaphoreGive(transaction_lock);
    return result;
}

esp_err_t ubx_set(uint8_t cls, uint8_t id, const uint8_t *payload, size_t length)
{
    xSemaphoreTake(transaction_lock, portMAX_DELAY);
    xQueueReset(rx_queue);
    esp_err_t result = send_message(cls, id, payload, length);
    const int64_t deadline_us = esp_timer_get_time() + REPLY_TIMEOUT_MS * 1000LL;
    while (result == ESP_OK) {
        const int64_t left_us = deadline_us - esp_timer_get_time();
        struct frame frame;
        if (left_us <= 0 || xQueueReceive(rx_queue, &frame, pdMS_TO_TICKS(left_us / 1000 + 1)) != pdTRUE) {
            result = ESP_ERR_TIMEOUT;
        } else if (frame.cls == CLASS_ACK && frame.length >= 2 && frame.payload[0] == cls && frame.payload[1] == id) {
            result = frame.id == ID_ACK_ACK ? ESP_OK : ESP_FAIL;
            break;
        }
    }
    xSemaphoreGive(transaction_lock);
    return result;
}

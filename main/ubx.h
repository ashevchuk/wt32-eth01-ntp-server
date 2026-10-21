#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#define UBX_MAX_PAYLOAD 80

esp_err_t ubx_init(uart_port_t port);

// Called for every valid frame, from the task that feeds the bytes, with a payload of any size the receiver
// sends (up to 800 bytes). It must not block. Replies to ubx_poll are limited to UBX_MAX_PAYLOAD.
typedef void (*ubx_handler_t)(uint8_t cls, uint8_t id, const uint8_t *payload, size_t length);
void ubx_set_handler(ubx_handler_t handler);

// Feeds one received byte. Returns true when the byte belongs to a UBX frame and must not be parsed as NMEA.
bool ubx_feed(uint8_t byte);

// Sends a poll request and returns the receiver's reply with the same class and id.
// ESP_ERR_TIMEOUT: no reply.
esp_err_t ubx_poll(uint8_t cls, uint8_t id, const uint8_t *payload, size_t length, uint8_t *reply,
                   size_t reply_size, size_t *reply_length);

// Sends a configuration message and waits for UBX-ACK. ESP_FAIL: the receiver answered with NAK;
// ESP_ERR_TIMEOUT: no answer.
esp_err_t ubx_set(uint8_t cls, uint8_t id, const uint8_t *payload, size_t length);

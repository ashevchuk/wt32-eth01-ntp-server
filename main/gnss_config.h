#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define GNSS_DELAY_CABLE_MAX_NS 32767
#define GNSS_DELAY_USER_LIMIT_NS 1000000

typedef struct {
    bool known;
    bool saved;
    uint8_t dynamic_model;
    int16_t cable_delay_ns;
    int32_t user_delay_ns;
    bool delays_managed;
} gnss_receiver_t;

// Applies the configuration in a background task shortly after boot.
esp_err_t gnss_config_start(void);

// Stores the delays and applies them to the receiver now; the receiver is written (and its memory saved)
// only when its current values differ, and *written tells whether that happened. ESP_ERR_INVALID_ARG: out of range; ESP_ERR_TIMEOUT / ESP_FAIL: the
// receiver did not answer or refused.
esp_err_t gnss_config_set_pps_delay(int16_t cable_delay_ns, int32_t user_delay_ns, bool *written);

void gnss_config_get(gnss_receiver_t *receiver);

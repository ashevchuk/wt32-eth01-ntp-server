#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

// Called from the PPS task right after a pulse, with the UTC second that pulse started
// and the esp_timer instant of the pulse.
typedef void (*gnss_second_cb_t)(const struct tm *utc, int64_t edge_us);

typedef struct {
    bool fix;
    bool has_position;
    uint8_t fix_mode;
    uint8_t fix_quality;
    uint8_t satellites_used;
    uint8_t satellites_in_view;
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float hdop;
    float pdop;
    float speed_kmh;
    float course_deg;
    int64_t updated_us;
} gnss_status_t;

void gnss_get_status(gnss_status_t *status);

esp_err_t gnss_start(gnss_second_cb_t on_second);

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

#define GNSS_MAX_SATELLITES 64
#define GNSS_UNKNOWN (-1)

typedef struct {
    char talker[3];
    uint8_t id;
    int16_t elevation_deg;
    int16_t azimuth_deg;
    int16_t snr_dbhz;
    bool used;
} gnss_satellite_t;

// Copies the satellites of the last complete GSV cycle and returns their number. Elevation, azimuth and signal
// strength are GNSS_UNKNOWN while the receiver does not report them; age_us is the time since the cycle ended.
size_t gnss_get_satellites(gnss_satellite_t satellites[GNSS_MAX_SATELLITES], int64_t *age_us);

// Statistics of the intervals between PPS pulses on the esp_timer clock over the last minute.
typedef struct {
    bool present;
    uint32_t pulses;
    uint8_t samples;
    float offset_ppm;
    float jitter_us;
    int32_t min_error_us;
    int32_t max_error_us;
} gnss_pps_t;

void gnss_get_pps(gnss_pps_t *pps);

esp_err_t gnss_start(gnss_second_cb_t on_second);

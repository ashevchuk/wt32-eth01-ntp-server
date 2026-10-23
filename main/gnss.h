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

#define GNSS_BAUD 115200

// Changes the speed of the receiver's UART on the ESP32 side only.
esp_err_t gnss_set_baud(uint32_t baud);

#define GNSS_MAX_SATELLITES 64
#define GNSS_UNKNOWN (-1)

// Satellites in the last UBX-NAV-SAT message. gnss_id: 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS.
// quality is the receiver's signal quality indicator: 0 none, 1 searching, 2 acquired, 3 detected but unusable,
// 4 code locked and time synchronised, 5-7 code and carrier locked.
typedef struct {
    uint8_t gnss_id;
    uint8_t id;
    int16_t elevation_deg;
    int16_t azimuth_deg;
    int16_t snr_dbhz;
    uint8_t quality;
    bool used;
    bool ephemeris;
    bool almanac;
} gnss_satellite_t;

#define GNSS_QUALITY_LOCKED 4

// Copies the satellites of the last NAV-SAT message and returns their number. Elevation, azimuth and signal
// strength are GNSS_UNKNOWN while the receiver does not report them; age_us is the time since the message.
size_t gnss_get_satellites(gnss_satellite_t satellites[GNSS_MAX_SATELLITES], int64_t *age_us);

// UBX-NAV-PVT: fix and time validity. time_accuracy_ns and horizontal_accuracy_m are negative while unknown.
// time_trusted tells whether the receiver's time is accepted as the GNSS source; known is false when no
// message arrived in the last seconds.
typedef struct {
    bool known;
    uint8_t fix_type;
    bool fix_ok;
    bool time_valid;
    bool time_resolved;
    bool time_trusted;
    uint8_t satellites;
    int64_t time_accuracy_ns;
    float horizontal_accuracy_m;
} gnss_pvt_t;

void gnss_get_pvt(gnss_pvt_t *pvt);

// UBX-MON-HW: antenna supervisor (0 init, 1 unknown, 2 ok, 3 short, 4 open), interference monitor
// (jamming_state 0 unknown, 1 ok, 2 warning, 3 critical; indicator 0-255) and the AGC level.
typedef struct {
    bool known;
    uint8_t antenna_status;
    uint8_t jamming_state;
    uint8_t jamming_indicator;
    float agc_percent;
    uint16_t noise_per_ms;
} gnss_hardware_t;

void gnss_get_hardware(gnss_hardware_t *hardware);

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

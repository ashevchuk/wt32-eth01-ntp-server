#pragma once

#include <stdint.h>
#include <time.h>

#include "esp_err.h"

typedef enum {
    NTP_CLOCK_UNSYNCHRONIZED,
    NTP_CLOCK_GNSS,
    NTP_CLOCK_NETWORK,
    NTP_CLOCK_LOCAL,
} ntp_clock_source_t;

typedef struct {
    ntp_clock_source_t source;
    int64_t unix_us;
    int64_t reference_unix_us;
    uint8_t stratum;
    uint8_t reference_id[4];
    int64_t root_delay_us;
    int64_t root_dispersion_us;
} ntp_clock_reading_t;

// The pulse that started the given UTC second was seen at edge_us on the esp_timer clock.
void ntp_clock_anchor_gnss(time_t unix_s, int64_t edge_us);

// Time that was set by hand or read from the RTC: it held unix_s at mono_us.
void ntp_clock_anchor_local(time_t unix_s, int64_t mono_us);

// Time obtained from an upstream NTP server: unix_us at mono_us. stratum is the one this server should report.
void ntp_clock_anchor_network(int64_t unix_us, int64_t mono_us, uint8_t stratum, const uint8_t reference_id[4],
                              int64_t root_delay_us);

// Starts the task that re-anchors to the RTC while neither GNSS nor the network is available.
esp_err_t ntp_clock_start(void);

// Time at the given esp_timer instant, extrapolated from the best available anchor.
void ntp_clock_read(int64_t mono_us, ntp_clock_reading_t *reading);

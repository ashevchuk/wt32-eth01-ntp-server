#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define LEAP_NONE 0
#define LEAP_INSERT 1
#define LEAP_DELETE 2

typedef struct {
    bool known;
    int8_t gps_utc_offset_s;
    int8_t change;
    bool event_known;
    int64_t seconds_to_event;
} leap_status_t;

// Starts the task that polls the receiver for the current and the scheduled leap second.
esp_err_t leap_start(void);

// NTP leap indicator for a reply made at the given esp_timer instant, when the clock read unix_s: non-zero only
// during the UTC day that ends with the scheduled leap second.
uint8_t leap_warning(int64_t mono_us, int64_t unix_s);

// Leap indicator from a scheduled change (+1, -1 or 0) and the seconds left until it happens.
uint8_t leap_indicator_at(int8_t change, int64_t unix_s, int64_t seconds_to_event);

void leap_get(leap_status_t *status);

#pragma once

#include <time.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t rtc_start(i2c_master_bus_handle_t bus);

// Returns ESP_ERR_INVALID_STATE when the oscillator-stopped flag is set, i.e. the time is not trustworthy.
esp_err_t rtc_read(struct tm *utc);

// Clears the oscillator-stopped flag on success.
esp_err_t rtc_write(const struct tm *utc);

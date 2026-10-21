#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "settings.h"

typedef struct {
    bool enabled;
    bool connected;
    int8_t rssi;
    char ip[16];
} wifi_status_t;

// Does nothing when Wi-Fi is disabled in the settings. Requires the default event loop.
esp_err_t wifi_start(const network_settings_t *network);
void wifi_get_status(wifi_status_t *status);

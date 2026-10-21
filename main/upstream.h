#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "settings.h"

typedef struct {
    bool enabled;
    bool synced;
    char server[SETTINGS_UPSTREAM_MAX_LENGTH];
    uint8_t server_stratum;
    int64_t age_s;
    int32_t delay_us;
    // How far this server's own time was from the upstream at the last sync; only known while GNSS was the source.
    bool has_offset;
    int64_t offset_us;
} upstream_status_t;

// Queries the configured server periodically and feeds the result to ntp_clock as the fallback source.
esp_err_t upstream_start(void);

// Empty disables the fallback. ESP_ERR_INVALID_ARG: not a host name or IPv4 address.
esp_err_t upstream_set_server(const char *server);

void upstream_get_status(upstream_status_t *status);

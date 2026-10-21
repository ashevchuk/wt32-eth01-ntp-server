#pragma once

#include "esp_err.h"
#include "settings.h"

// Serves the munin-node protocol on TCP 4949 with a single plugin, one field per sensor.
esp_err_t munin_start(void);

void munin_get_plugin_name(char plugin[SETTINGS_MUNIN_PLUGIN_MAX_LENGTH]);

// Takes effect for the next request; ESP_ERR_INVALID_ARG unless it is a valid Munin identifier.
esp_err_t munin_set_plugin_name(const char *plugin);

#pragma once

#include "esp_err.h"

#include "settings.h"

#define DISPLAY_ROTATE_MIN_S 2
#define DISPLAY_ROTATE_MAX_S 3600

esp_err_t bigclock_start(void);

void bigclock_get_display(display_settings_t *display);

// Stores the settings and applies them at once; ESP_ERR_INVALID_ARG when the rotation period is out of range or no screen is selected.
esp_err_t bigclock_set_display(const display_settings_t *display);

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "settings.h"

#define SENSORS_MAX SETTINGS_MAX_SENSORS
#define SENSOR_ID_LENGTH 17

typedef struct {
    char id[SENSOR_ID_LENGTH];
    char name[SETTINGS_SENSOR_NAME_MAX_LENGTH];
    char label[SETTINGS_SENSOR_NAME_MAX_LENGTH];
    bool present;
    bool has_reading;
    bool munin;
    float temperature_c;
} sensor_info_t;

// Scans the 1-Wire bus periodically and reads every DS18B20 on it.
esp_err_t sensors_start(void);

// Sensors seen since boot, in discovery order. A reading older than a few intervals is reported as missing.
size_t sensors_snapshot(sensor_info_t out[SENSORS_MAX]);

// A name is a valid Munin field name: letters, digits and underscore, not starting with a digit.
bool sensors_name_valid(const char *name);

// An empty name restores the default derived from the ROM code.
// ESP_ERR_NOT_FOUND: unknown id; ESP_ERR_INVALID_ARG: bad name; ESP_ERR_INVALID_STATE: name already used.
esp_err_t sensors_rename(const char *id, const char *name);

// A label is the text Munin shows for the sensor: up to 31 printable ASCII characters without quotes or backslash.
bool sensors_label_valid(const char *label);

// An empty label makes Munin show the name. ESP_ERR_NOT_FOUND: unknown id; ESP_ERR_INVALID_ARG: bad label.
esp_err_t sensors_set_label(const char *id, const char *label);

// Publishes the sensor in Munin or stops doing so. ESP_ERR_NOT_FOUND: unknown id.
esp_err_t sensors_set_munin(const char *id, bool enabled);

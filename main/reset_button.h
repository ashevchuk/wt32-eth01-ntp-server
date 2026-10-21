#pragma once

#include "esp_err.h"

// Holding the button restores DHCP on Ethernet and restarts the device.
esp_err_t reset_button_start(void);

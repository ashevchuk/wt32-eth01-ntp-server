#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t ntp_server_start(void);
uint32_t ntp_server_request_count(void);

#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

// POST /api/ota: the raw firmware image is the request body.
esp_err_t ota_upload_handler(httpd_req_t *req);

// With app rollback enabled a freshly installed image must confirm itself, otherwise the bootloader reverts it.
esp_err_t ota_confirm_running_image(void);

const char *ota_running_partition_label(void);

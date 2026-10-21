#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t web_start(void);

// Sends {"ok":...,"message":...} with the given HTTP status line, e.g. HTTPD_400.
esp_err_t web_respond(httpd_req_t *req, const char *status, const char *message);

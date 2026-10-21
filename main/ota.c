#include "ota.h"

#include <string.h>

#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "web.h"

#define CHUNK_SIZE 1024
#define MAX_CONSECUTIVE_TIMEOUTS 5
#define REBOOT_DELAY_MS 500
#define HTTP_STATUS_TOO_LARGE "413 Content Too Large"
#define APP_DESC_OFFSET (sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t))

static const char *TAG = "ota";

static uint8_t chunk[CHUNK_SIZE];

// Fills the buffer completely; false on a socket error or after too many consecutive timeouts.
static bool receive_exactly(httpd_req_t *req, uint8_t *buffer, size_t length)
{
    size_t received = 0;
    int timeouts = 0;
    while (received < length) {
        const int n = httpd_req_recv(req, (char *)buffer + received, length - received);
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < MAX_CONSECUTIVE_TIMEOUTS) {
            continue;
        }
        if (n <= 0) {
            return false;
        }
        timeouts = 0;
        received += (size_t)n;
    }
    return true;
}

// Rejects things like a bootloader or partition table image that would brick the slot.
static bool is_firmware_of_this_project(const uint8_t *first_chunk, size_t length)
{
    if (length < APP_DESC_OFFSET + sizeof(esp_app_desc_t) || first_chunk[0] != ESP_IMAGE_HEADER_MAGIC) {
        return false;
    }
    esp_app_desc_t incoming;
    memcpy(&incoming, first_chunk + APP_DESC_OFFSET, sizeof(incoming));
    return incoming.magic_word == ESP_APP_DESC_MAGIC_WORD
           && strncmp(incoming.project_name, esp_app_get_description()->project_name, sizeof(incoming.project_name)) == 0;
}

esp_err_t ota_upload_handler(httpd_req_t *req)
{
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        return web_respond(req, HTTPD_500, "no OTA partition");
    }
    if (req->content_len == 0) {
        return web_respond(req, HTTPD_400, "empty request");
    }
    if (req->content_len > target->size) {
        return web_respond(req, HTTP_STATUS_TOO_LARGE, "image does not fit the slot");
    }

    size_t remaining = req->content_len;
    size_t length = remaining < CHUNK_SIZE ? remaining : CHUNK_SIZE;
    if (!receive_exactly(req, chunk, length)) {
        return web_respond(req, HTTPD_400, "upload interrupted");
    }
    if (!is_firmware_of_this_project(chunk, length)) {
        return web_respond(req, HTTPD_400, "not a firmware image of this project");
    }

    esp_ota_handle_t handle;
    if (esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle) != ESP_OK) {
        return web_respond(req, HTTPD_500, "could not start the update");
    }
    while (true) {
        if (esp_ota_write(handle, chunk, length) != ESP_OK) {
            esp_ota_abort(handle);
            return web_respond(req, HTTPD_500, "flash write failed");
        }
        remaining -= length;
        if (remaining == 0) {
            break;
        }
        length = remaining < CHUNK_SIZE ? remaining : CHUNK_SIZE;
        if (!receive_exactly(req, chunk, length)) {
            esp_ota_abort(handle);
            return web_respond(req, HTTPD_400, "upload interrupted");
        }
    }

    if (esp_ota_end(handle) != ESP_OK) {
        return web_respond(req, HTTPD_400, "image validation failed");
    }
    if (esp_ota_set_boot_partition(target) != ESP_OK) {
        return web_respond(req, HTTPD_500, "could not select the new image");
    }
    ESP_LOGI(TAG, "installed into %s, restarting", target->label);
    web_respond(req, HTTPD_200, "update installed, restarting");
    vTaskDelay(pdMS_TO_TICKS(REBOOT_DELAY_MS));
    esp_restart();
}

esp_err_t ota_confirm_running_image(void)
{
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK
        && state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG, "confirming the new image");
        return esp_ota_mark_app_valid_cancel_rollback();
    }
    return ESP_OK;
}

const char *ota_running_partition_label(void)
{
    return esp_ota_get_running_partition()->label;
}

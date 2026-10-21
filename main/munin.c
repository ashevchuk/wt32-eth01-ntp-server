#include "munin.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "sensors.h"

#define MUNIN_PORT 4949
#define LINE_SIZE 128
#define REPLY_SIZE 1536
#define CLIENT_TIMEOUT_S 30
#define LISTEN_BACKLOG 1
#define ETH_NETIF_KEY "ETH_DEF"
#define NODE_VERSION "1.0"
#define NO_VALUE "U"

static const char *TAG = "munin";

static char plugin_name[SETTINGS_MUNIN_PLUGIN_MAX_LENGTH];
static portMUX_TYPE plugin_lock = portMUX_INITIALIZER_UNLOCKED;

void munin_get_plugin_name(char plugin[SETTINGS_MUNIN_PLUGIN_MAX_LENGTH])
{
    portENTER_CRITICAL(&plugin_lock);
    memcpy(plugin, plugin_name, sizeof(plugin_name));
    portEXIT_CRITICAL(&plugin_lock);
}

esp_err_t munin_set_plugin_name(const char *plugin)
{
    // Plugin names follow the same rule as field names.
    ESP_RETURN_ON_FALSE(sensors_name_valid(plugin), ESP_ERR_INVALID_ARG, TAG, "invalid plugin name");
    ESP_RETURN_ON_ERROR(settings_save_munin_plugin(plugin), TAG, "store plugin name");
    portENTER_CRITICAL(&plugin_lock);
    snprintf(plugin_name, sizeof(plugin_name), "%s", plugin);
    portEXIT_CRITICAL(&plugin_lock);
    return ESP_OK;
}

static void send_text(int client, const char *text)
{
    send(client, text, strlen(text), 0);
}

static const char *hostname(void)
{
    const char *name = NULL;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey(ETH_NETIF_KEY);
    if (netif && esp_netif_get_hostname(netif, &name) == ESP_OK && name) {
        return name;
    }
    return "ntp-server";
}

static void reply_list(int client)
{
    sensor_info_t sensors[SENSORS_MAX];
    char reply[LINE_SIZE];
    char plugin[SETTINGS_MUNIN_PLUGIN_MAX_LENGTH];
    munin_get_plugin_name(plugin);
    snprintf(reply, sizeof(reply), "%s\n", sensors_snapshot(sensors) > 0 ? plugin : "");
    send_text(client, reply);
}

static void reply_config(int client)
{
    sensor_info_t sensors[SENSORS_MAX];
    const size_t count = sensors_snapshot(sensors);
    char reply[REPLY_SIZE];
    int length = snprintf(reply, sizeof(reply),
                          "graph_title Temperature\n"
                          "graph_vlabel degrees Celsius\n"
                          "graph_category sensors\n"
                          "graph_args --base 1000\n");
    for (size_t i = 0; i < count && length < (int)sizeof(reply); i++) {
        if (!sensors[i].name[0]) {
            continue;
        }
        length += snprintf(reply + length, sizeof(reply) - length, "%s.label %s\n%s.info DS18B20 %s\n",
                           sensors[i].name, sensors[i].name, sensors[i].name, sensors[i].id);
    }
    length += snprintf(reply + length, sizeof(reply) - length, ".\n");
    send(client, reply, length < (int)sizeof(reply) ? length : (int)sizeof(reply) - 1, 0);
}

static void reply_fetch(int client)
{
    sensor_info_t sensors[SENSORS_MAX];
    const size_t count = sensors_snapshot(sensors);
    char reply[REPLY_SIZE];
    int length = 0;
    for (size_t i = 0; i < count && length < (int)sizeof(reply); i++) {
        if (!sensors[i].name[0]) {
            continue;
        }
        if (sensors[i].has_reading) {
            length += snprintf(reply + length, sizeof(reply) - length, "%s.value %.2f\n", sensors[i].name,
                               sensors[i].temperature_c);
        } else {
            length += snprintf(reply + length, sizeof(reply) - length, "%s.value " NO_VALUE "\n", sensors[i].name);
        }
    }
    length += snprintf(reply + length, sizeof(reply) - length, ".\n");
    send(client, reply, length < (int)sizeof(reply) ? length : (int)sizeof(reply) - 1, 0);
}

// Returns false when the client asked to close the connection.
static bool handle_command(int client, char *line)
{
    char *argument = strchr(line, ' ');
    if (argument) {
        *argument++ = '\0';
    }
    char plugin[SETTINGS_MUNIN_PLUGIN_MAX_LENGTH];
    munin_get_plugin_name(plugin);
    const bool plugin_ok = argument && strcmp(argument, plugin) == 0;
    char reply[LINE_SIZE];

    if (strcmp(line, "list") == 0) {
        reply_list(client);
    } else if (strcmp(line, "nodes") == 0) {
        snprintf(reply, sizeof(reply), "%s\n.\n", hostname());
        send_text(client, reply);
    } else if (strcmp(line, "config") == 0 && plugin_ok) {
        reply_config(client);
    } else if (strcmp(line, "fetch") == 0 && plugin_ok) {
        reply_fetch(client);
    } else if (strcmp(line, "version") == 0) {
        snprintf(reply, sizeof(reply), "munin node on %s version: " NODE_VERSION "\n", hostname());
        send_text(client, reply);
    } else if (strcmp(line, "cap") == 0) {
        send_text(client, "cap\n");
    } else if (strcmp(line, "quit") == 0 || strcmp(line, ".") == 0) {
        return false;
    } else if (strcmp(line, "config") == 0 || strcmp(line, "fetch") == 0) {
        send_text(client, "# Unknown service\n.\n");
    } else {
        send_text(client, "# Unknown command. Try list, nodes, config, fetch, version or quit\n");
    }
    return true;
}

static void serve_client(int client)
{
    const struct timeval timeout = {.tv_sec = CLIENT_TIMEOUT_S};
    setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    char greeting[LINE_SIZE];
    snprintf(greeting, sizeof(greeting), "# munin node at %s\n", hostname());
    send_text(client, greeting);

    char line[LINE_SIZE];
    size_t length = 0;
    while (true) {
        char byte;
        if (recv(client, &byte, 1, 0) != 1) {
            return;
        }
        if (byte == '\n') {
            line[length] = '\0';
            if (!handle_command(client, line)) {
                return;
            }
            length = 0;
        } else if (byte != '\r' && length < sizeof(line) - 1) {
            line[length++] = byte;
        }
    }
}

static void server_task(void *arg)
{
    const int listener = (int)(intptr_t)arg;
    while (true) {
        const int client = accept(listener, NULL, NULL);
        if (client < 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        serve_client(client);
        close(client);
    }
}

esp_err_t munin_start(void)
{
    ESP_RETURN_ON_ERROR(settings_load_munin_plugin(plugin_name), TAG, "load plugin name");
    const int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    ESP_RETURN_ON_FALSE(listener >= 0, ESP_FAIL, TAG, "socket: errno %d", errno);
    const struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(MUNIN_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(listener, (const struct sockaddr *)&address, sizeof(address)) < 0 || listen(listener, LISTEN_BACKLOG) < 0) {
        ESP_LOGE(TAG, "bind/listen: errno %d", errno);
        close(listener);
        return ESP_FAIL;
    }
    ESP_RETURN_ON_FALSE(xTaskCreate(server_task, "munin", 4096, (void *)(intptr_t)listener, 3, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    ESP_LOGI(TAG, "listening on tcp/%d", MUNIN_PORT);
    return ESP_OK;
}

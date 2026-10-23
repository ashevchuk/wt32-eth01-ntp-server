#include "munin.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "gnss.h"
#include "sensors.h"

#define MUNIN_PORT 4949
#define LINE_SIZE 128
#define REPLY_SIZE 1536
#define CLIENT_TIMEOUT_S 30
#define LISTEN_BACKLOG 1
#define ETH_NETIF_KEY "ETH_DEF"
#define NODE_VERSION "1.0"
#define NO_VALUE "U"
#define TASK_STACK_SIZE 6144

static const char *TAG = "munin";

static char plugin_name[SETTINGS_MUNIN_PLUGIN_MAX_LENGTH];
static portMUX_TYPE plugin_lock = portMUX_INITIALIZER_UNLOCKED;

// Readings older than this mean the receiver has stopped talking.
#define GNSS_STALE_US 5000000
#define NO_DOP_VALUE 99.0f
#define LIST_SIZE 192
#define FETCH_SIZE 256

struct builtin_plugin {
    const char *name;
    const char *config;
    int (*fetch)(char *out, size_t size);
};

static bool gnss_fresh(const gnss_status_t *status)
{
    return status->updated_us && esp_timer_get_time() - status->updated_us < GNSS_STALE_US;
}

static int fetch_satellites(char *out, size_t size)
{
    gnss_satellite_t satellites[GNSS_MAX_SATELLITES];
    int64_t age_us;
    const size_t count = gnss_get_satellites(satellites, &age_us);
    if (age_us > GNSS_STALE_US) {
        return snprintf(out, size, "in_view.value " NO_VALUE "\ntracked.value " NO_VALUE "\nlocked.value " NO_VALUE
                                   "\nused.value " NO_VALUE "\nephemeris.value " NO_VALUE "\n");
    }
    int tracked = 0, locked = 0, used = 0, ephemeris = 0;
    for (size_t i = 0; i < count; i++) {
        tracked += satellites[i].snr_dbhz != GNSS_UNKNOWN;
        locked += satellites[i].quality >= GNSS_QUALITY_LOCKED;
        used += satellites[i].used;
        ephemeris += satellites[i].ephemeris;
    }
    return snprintf(out, size, "in_view.value %u\ntracked.value %d\nlocked.value %d\nused.value %d\nephemeris.value %d\n",
                    (unsigned)count, tracked, locked, used, ephemeris);
}

static int fetch_signal(char *out, size_t size)
{
    gnss_satellite_t satellites[GNSS_MAX_SATELLITES];
    int64_t age_us;
    const size_t count = gnss_get_satellites(satellites, &age_us);
    int tracked = 0, sum = 0, strongest = 0;
    for (size_t i = 0; i < count; i++) {
        if (satellites[i].snr_dbhz != GNSS_UNKNOWN) {
            tracked++;
            sum += satellites[i].snr_dbhz;
            strongest = satellites[i].snr_dbhz > strongest ? satellites[i].snr_dbhz : strongest;
        }
    }
    if (age_us > GNSS_STALE_US || tracked == 0) {
        return snprintf(out, size, "mean.value " NO_VALUE "\nmax.value " NO_VALUE "\n");
    }
    return snprintf(out, size, "mean.value %.1f\nmax.value %d\n", (double)sum / tracked, strongest);
}

// The receiver reports 99.99 while it has no fix.
static int fetch_dop(char *out, size_t size)
{
    gnss_status_t status;
    gnss_get_status(&status);
    const bool usable = gnss_fresh(&status) && status.hdop < NO_DOP_VALUE && status.pdop < NO_DOP_VALUE;
    if (!usable) {
        return snprintf(out, size, "hdop.value " NO_VALUE "\npdop.value " NO_VALUE "\n");
    }
    return snprintf(out, size, "hdop.value %.2f\npdop.value %.2f\n", status.hdop, status.pdop);
}

static int fetch_pps_offset(char *out, size_t size)
{
    gnss_pps_t pps;
    gnss_get_pps(&pps);
    if (!pps.present || pps.samples == 0) {
        return snprintf(out, size, "offset.value " NO_VALUE "\n");
    }
    return snprintf(out, size, "offset.value %.2f\n", pps.offset_ppm);
}

static int fetch_pps_jitter(char *out, size_t size)
{
    gnss_pps_t pps;
    gnss_get_pps(&pps);
    if (!pps.present || pps.samples == 0) {
        return snprintf(out, size, "jitter.value " NO_VALUE "\n");
    }
    return snprintf(out, size, "jitter.value %.2f\n", pps.jitter_us);
}

static int fetch_time_accuracy(char *out, size_t size)
{
    gnss_pvt_t pvt;
    gnss_get_pvt(&pvt);
    if (!pvt.known || pvt.time_accuracy_ns < 0) {
        return snprintf(out, size, "accuracy.value " NO_VALUE "\n");
    }
    return snprintf(out, size, "accuracy.value %lld\n", (long long)pvt.time_accuracy_ns);
}

static int fetch_interference(char *out, size_t size)
{
    gnss_hardware_t hardware;
    gnss_get_hardware(&hardware);
    if (!hardware.known) {
        return snprintf(out, size, "jamming.value " NO_VALUE "\n");
    }
    return snprintf(out, size, "jamming.value %u\n", hardware.jamming_indicator);
}

static int fetch_agc(char *out, size_t size)
{
    gnss_hardware_t hardware;
    gnss_get_hardware(&hardware);
    if (!hardware.known) {
        return snprintf(out, size, "agc.value " NO_VALUE "\n");
    }
    return snprintf(out, size, "agc.value %.1f\n", hardware.agc_percent);
}

static int fetch_jamming_state(char *out, size_t size)
{
    gnss_hardware_t hardware;
    gnss_get_hardware(&hardware);
    // 0 means the monitor has no verdict yet, so it is not a reading.
    if (!hardware.known || hardware.jamming_state == 0) {
        return snprintf(out, size, "state.value " NO_VALUE "\n");
    }
    return snprintf(out, size, "state.value %u\n", hardware.jamming_state);
}

static int fetch_spoofing(char *out, size_t size)
{
    gnss_module_t module;
    gnss_get_module(&module);
    // 0 means unknown or not supported by this protocol version, so it is not a reading.
    if (!module.status_known || module.spoofing_state == 0) {
        return snprintf(out, size, "state.value " NO_VALUE "\n");
    }
    return snprintf(out, size, "state.value %u\n", module.spoofing_state);
}

static const struct builtin_plugin builtin_plugins[] = {
    {"gnss_satellites",
     "graph_title GNSS satellites\n"
     "graph_vlabel satellites\n"
     "graph_category gnss\n"
     "graph_args --base 1000 --lower-limit 0\n"
     "in_view.label Reported by the receiver\n"
     "tracked.label With a signal\n"
     "locked.label Locked and time synchronised\n"
     "ephemeris.label With ephemeris\n"
     "used.label Used in fix\n",
     fetch_satellites},
    {"gnss_signal",
     "graph_title GNSS signal strength\n"
     "graph_vlabel dB-Hz\n"
     "graph_category gnss\n"
     "graph_args --base 1000 --lower-limit 0\n"
     "mean.label Mean of tracked satellites\n"
     "max.label Strongest satellite\n",
     fetch_signal},
    {"gnss_dop",
     "graph_title GNSS dilution of precision\n"
     "graph_vlabel DOP\n"
     "graph_category gnss\n"
     "graph_args --base 1000 --lower-limit 0\n"
     "hdop.label HDOP\n"
     "pdop.label PDOP\n",
     fetch_dop},
    {"gnss_pps_offset",
     "graph_title PPS interval offset\n"
     "graph_vlabel ppm\n"
     "graph_category gnss\n"
     "graph_args --base 1000\n"
     "offset.label ESP32 clock offset\n"
     "offset.info Mean error of the pulse interval over the last minute; positive means the ESP32 clock runs fast\n",
     fetch_pps_offset},
    {"gnss_pps_jitter",
     "graph_title PPS jitter\n"
     "graph_vlabel microseconds\n"
     "graph_category gnss\n"
     "graph_args --base 1000 --lower-limit 0\n"
     "jitter.label Standard deviation of the pulse interval\n",
     fetch_pps_jitter},
    {"gnss_time_accuracy",
     "graph_title GNSS time accuracy estimate\n"
     "graph_vlabel ns\n"
     "graph_category gnss\n"
     "graph_args --base 1000 --lower-limit 0\n"
     "accuracy.label Estimated time error\n"
     "accuracy.info Receiver's estimate of its time error; unknown without a fix\n",
     fetch_time_accuracy},
    {"gnss_interference",
     "graph_title GNSS interference indicator\n"
     "graph_vlabel 0 - 255\n"
     "graph_category gnss\n"
     "graph_args --base 1000 --lower-limit 0 --upper-limit 255\n"
     "jamming.label Continuous-wave interference\n"
     "jamming.info 0 means none, 255 strong\n",
     fetch_interference},
    {"gnss_jamming_state",
     "graph_title GNSS interference state\n"
     "graph_vlabel 1 ok, 2 warning, 3 critical\n"
     "graph_category gnss\n"
     "graph_args --base 1000 --lower-limit 0 --upper-limit 3\n"
     "state.label Verdict of the interference monitor\n"
     "state.info Warning: interference is visible but the fix holds; critical: interference and no fix\n"
     "state.warning :1\n"
     "state.critical :2\n",
     fetch_jamming_state},
    {"gnss_spoofing",
     "graph_title GNSS spoofing detection\n"
     "graph_vlabel 1 none, 2 indicated, 3 several\n"
     "graph_category gnss\n"
     "graph_args --base 1000 --lower-limit 0 --upper-limit 3\n"
     "state.label Verdict of the spoofing detection\n"
     "state.info Heuristic of the receiver, not a guarantee; unknown on protocol 15\n"
     "state.warning :1\n"
     "state.critical :2\n",
     fetch_spoofing},
    {"gnss_agc",
     "graph_title GNSS receiver AGC\n"
     "graph_vlabel percent\n"
     "graph_category gnss\n"
     "graph_args --base 1000 --lower-limit 0 --upper-limit 100\n"
     "agc.label Gain control level\n",
     fetch_agc},
};
#define BUILTIN_PLUGIN_COUNT (sizeof(builtin_plugins) / sizeof(builtin_plugins[0]))

static const struct builtin_plugin *find_builtin_plugin(const char *name)
{
    for (size_t i = 0; i < BUILTIN_PLUGIN_COUNT; i++) {
        if (strcmp(builtin_plugins[i].name, name) == 0) {
            return &builtin_plugins[i];
        }
    }
    return NULL;
}

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
    ESP_RETURN_ON_FALSE(!find_builtin_plugin(plugin), ESP_ERR_INVALID_STATE, TAG, "name of a built-in plugin");
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
    const size_t sensor_count = sensors_snapshot(sensors);
    bool any_named = false;
    for (size_t i = 0; i < sensor_count; i++) {
        any_named = any_named || sensors[i].munin;
    }
    char plugin[SETTINGS_MUNIN_PLUGIN_MAX_LENGTH];
    munin_get_plugin_name(plugin);

    char reply[LIST_SIZE];
    int length = snprintf(reply, sizeof(reply), "%s", any_named ? plugin : "");
    for (size_t i = 0; i < BUILTIN_PLUGIN_COUNT; i++) {
        length += snprintf(reply + length, sizeof(reply) - length, "%s%s", length ? " " : "", builtin_plugins[i].name);
    }
    snprintf(reply + length, sizeof(reply) - length, "\n");
    send_text(client, reply);
}

static void reply_builtin_fetch(int client, const struct builtin_plugin *plugin)
{
    char reply[FETCH_SIZE];
    int length = plugin->fetch(reply, sizeof(reply) - sizeof(".\n"));
    length += snprintf(reply + length, sizeof(reply) - length, ".\n");
    send(client, reply, length, 0);
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
        if (!sensors[i].munin) {
            continue;
        }
        length += snprintf(reply + length, sizeof(reply) - length, "%s.label %s\n%s.info DS18B20 %s\n", sensors[i].name,
                           sensors[i].label[0] ? sensors[i].label : sensors[i].name, sensors[i].name, sensors[i].id);
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
        if (!sensors[i].munin) {
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
    const bool sensor_plugin = argument && strcmp(argument, plugin) == 0;
    const struct builtin_plugin *builtin = argument ? find_builtin_plugin(argument) : NULL;
    char reply[LINE_SIZE];

    if (strcmp(line, "list") == 0) {
        reply_list(client);
    } else if (strcmp(line, "nodes") == 0) {
        snprintf(reply, sizeof(reply), "%s\n.\n", hostname());
        send_text(client, reply);
    } else if (strcmp(line, "config") == 0 && sensor_plugin) {
        reply_config(client);
    } else if (strcmp(line, "config") == 0 && builtin) {
        send_text(client, builtin->config);
        send_text(client, ".\n");
    } else if (strcmp(line, "fetch") == 0 && sensor_plugin) {
        reply_fetch(client);
    } else if (strcmp(line, "fetch") == 0 && builtin) {
        reply_builtin_fetch(client, builtin);
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
    ESP_RETURN_ON_FALSE(xTaskCreate(server_task, "munin", TASK_STACK_SIZE, (void *)(intptr_t)listener, 3, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    ESP_LOGI(TAG, "listening on tcp/%d", MUNIN_PORT);
    return ESP_OK;
}

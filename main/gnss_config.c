#include "gnss_config.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gnss.h"
#include "settings.h"
#include "ubx.h"

#define CLASS_CFG 0x06
#define CLASS_MON 0x0A
#define ID_MON_VER 0x04
#define ID_CFG_NAV5 0x24
#define ID_CFG_TP5 0x31
#define ID_CFG_CFG 0x09
#define ID_CFG_MSG 0x01
#define ID_CFG_PRT 0x00
#define ID_CFG_ITFM 0x39
#define ID_CFG_RATE 0x08
#define ID_CFG_NMEA 0x17
#define ID_CFG_GNSS 0x3E

#define NAV5_PAYLOAD_SIZE 36
#define NAV5_MASK_DYNMODEL 0x0001
#define NAV5_OFFSET_MASK 0
#define NAV5_OFFSET_DYNMODEL 2
#define DYNAMIC_MODEL_STATIONARY 2

#define TP5_PAYLOAD_SIZE 32
#define TP5_TIME_PULSE_INDEX 0
#define TP5_OFFSET_PERIOD 8
#define TP5_OFFSET_PERIOD_LOCKED 12
#define TP5_OFFSET_LENGTH_LOCKED 20
#define TP5_OFFSET_FLAGS 28
#define TP5_OFFSET_CABLE_DELAY 4
#define TP5_OFFSET_USER_DELAY 24

// One pulse per second, 100 ms long once locked, rising edge on the UTC second, absent while the receiver is
// not locked to GNSS time.
#define TP5_PERIOD_US 1000000
#define TP5_PULSE_LENGTH_US 100000
#define TP5_FLAGS_MASK 0x07FF
#define TP5_FLAGS_WANTED 0x0077

#define RATE_PAYLOAD_SIZE 6
#define RATE_MEASUREMENT_MS 1000
#define RATE_SOLUTIONS_PER_MEASUREMENT 1

#define NMEA_VERSION_OFFSET 1
#define NMEA_VERSION_4_0 0x40
#define NMEA_MAX_PAYLOAD_SIZE 20

#define GNSS_HEADER_SIZE 4
#define GNSS_OFFSET_BLOCK_COUNT 3
#define GNSS_BLOCK_SIZE 8
#define GNSS_BLOCK_OFFSET_ID 0
#define GNSS_BLOCK_OFFSET_FLAGS 4
#define GNSS_ID_GALILEO 2
#define GNSS_FLAG_ENABLE 0x00000001u
#define GNSS_SIGNALS_MASK 0x00FF0000u
#define GNSS_SIGNAL_GALILEO_E1 0x00010000u

#define ITFM_PAYLOAD_SIZE 8
#define ITFM_ENABLE (1u << 31)
#define ITFM_OFFSET_CONFIG2 4
#define ITFM_ANTENNA_SHIFT 12
#define ITFM_ANTENNA_MASK (3u << ITFM_ANTENNA_SHIFT)
#define ITFM_ANTENNA_ACTIVE 2u

#define PRT_PAYLOAD_SIZE 20
#define PRT_PORT_UART1 1
#define PRT_OFFSET_BAUD_RATE 8
#define PRT_OFFSET_OUT_PROTOCOLS 14
#define PRT_PROTOCOL_UBX 0x0001
#define PRT_PROTOCOL_NMEA 0x0002

#define CFG_MSG_REPLY_SIZE 8
#define CFG_MSG_OFFSET_UART1_RATE 3
#define CFG_MSG_SET_SIZE 3
#define MESSAGE_RATE 1

#define CFG_ALL_SECTIONS 0x0000FFFF
#define CFG_DEVICE_BBR_AND_FLASH 0x03
#define CFG_PAYLOAD_SIZE 13

#define START_DELAY_MS 3000
// The receiver needs a moment to apply a changed port setup before it answers again.
#define PORT_RECONFIGURE_MS 300
#define GNSS_RESTART_MS 1000
#define TASK_STACK_SIZE 4096
#define TASK_PRIORITY 3

static const char *TAG = "gnss_config";

static gnss_receiver_t receiver;
static portMUX_TYPE receiver_lock = portMUX_INITIALIZER_UNLOCKED;

static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

static void put_u32(uint8_t *p, uint32_t value)
{
    for (int i = 0; i < 4; i++) {
        p[i] = (uint8_t)(value >> (8 * i));
    }
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static esp_err_t poll_nav5(uint8_t payload[NAV5_PAYLOAD_SIZE])
{
    size_t length;
    ESP_RETURN_ON_ERROR(ubx_poll(CLASS_CFG, ID_CFG_NAV5, NULL, 0, payload, NAV5_PAYLOAD_SIZE, &length), TAG, "poll NAV5");
    return length == NAV5_PAYLOAD_SIZE ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

static esp_err_t poll_tp5(uint8_t payload[TP5_PAYLOAD_SIZE])
{
    const uint8_t request[] = {TP5_TIME_PULSE_INDEX};
    size_t length;
    ESP_RETURN_ON_ERROR(ubx_poll(CLASS_CFG, ID_CFG_TP5, request, sizeof(request), payload, TP5_PAYLOAD_SIZE, &length),
                        TAG, "poll TP5");
    return length == TP5_PAYLOAD_SIZE ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

static esp_err_t ensure_stationary(bool *changed)
{
    uint8_t nav5[NAV5_PAYLOAD_SIZE];
    ESP_RETURN_ON_ERROR(poll_nav5(nav5), TAG, "read NAV5");
    if (nav5[NAV5_OFFSET_DYNMODEL] != DYNAMIC_MODEL_STATIONARY) {
        put_u16(nav5 + NAV5_OFFSET_MASK, NAV5_MASK_DYNMODEL);
        nav5[NAV5_OFFSET_DYNMODEL] = DYNAMIC_MODEL_STATIONARY;
        ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_NAV5, nav5, sizeof(nav5)), TAG, "write NAV5");
        *changed = true;
    }
    return ESP_OK;
}

// Without the interference monitor the receiver reports its jamming state as unknown, and it judges the signal level
// differently for an active and a passive antenna. The antenna here is active. The thresholds stay as the receiver has them.
static esp_err_t ensure_interference_monitor(bool *changed)
{
    uint8_t itfm[ITFM_PAYLOAD_SIZE];
    size_t length;
    ESP_RETURN_ON_ERROR(ubx_poll(CLASS_CFG, ID_CFG_ITFM, NULL, 0, itfm, sizeof(itfm), &length), TAG, "read interference monitor");
    ESP_RETURN_ON_FALSE(length == ITFM_PAYLOAD_SIZE, ESP_ERR_INVALID_SIZE, TAG, "interference monitor reply");
    const uint32_t config = get_u32(itfm);
    const uint32_t config2 = get_u32(itfm + ITFM_OFFSET_CONFIG2);
    const bool antenna_active = (config2 & ITFM_ANTENNA_MASK) == (ITFM_ANTENNA_ACTIVE << ITFM_ANTENNA_SHIFT);
    if (!(config & ITFM_ENABLE) || !antenna_active) {
        put_u32(itfm, config | ITFM_ENABLE);
        put_u32(itfm + ITFM_OFFSET_CONFIG2, (config2 & ~ITFM_ANTENNA_MASK) | (ITFM_ANTENNA_ACTIVE << ITFM_ANTENNA_SHIFT));
        ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_ITFM, itfm, sizeof(itfm)), TAG, "write interference monitor");
        *changed = true;
    }
    return ESP_OK;
}

// Periodic UBX messages are only sent on a port that has UBX enabled for output; polls are answered either way.
// NMEA output is needed for the sentences below.
static esp_err_t ensure_ubx_output(bool *changed)
{
    const uint8_t request[] = {PRT_PORT_UART1};
    uint8_t port[PRT_PAYLOAD_SIZE];
    size_t length;
    ESP_RETURN_ON_ERROR(ubx_poll(CLASS_CFG, ID_CFG_PRT, request, sizeof(request), port, sizeof(port), &length), TAG,
                        "read port");
    ESP_RETURN_ON_FALSE(length == PRT_PAYLOAD_SIZE, ESP_ERR_INVALID_SIZE, TAG, "port reply");
    const uint16_t protocols = get_u16(port + PRT_OFFSET_OUT_PROTOCOLS);
    const uint16_t wanted = PRT_PROTOCOL_UBX | PRT_PROTOCOL_NMEA;
    if ((protocols & wanted) != wanted) {
        put_u16(port + PRT_OFFSET_OUT_PROTOCOLS, protocols | wanted);
        ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_PRT, port, sizeof(port)), TAG, "write port");
        vTaskDelay(pdMS_TO_TICKS(PORT_RECONFIGURE_MS));
        *changed = true;
    }
    return ESP_OK;
}

// The receiver sends these once per navigation solution on this UART: satellites (UBX-NAV-SAT), time and fix
// validity (UBX-NAV-PVT), the antenna and interference monitor (UBX-MON-HW) and its start-up state (UBX-NAV-STATUS),
// plus the NMEA sentences GGA, GSA, GSV and RMC that gnss.c parses.
static const struct {
    uint8_t cls;
    uint8_t id;
} PERIODIC_MESSAGES[] = {{0x01, 0x35}, {0x01, 0x07}, {0x0A, 0x09}, {0x01, 0x03},
                                                                    {0xF0, 0x00}, {0xF0, 0x02}, {0xF0, 0x03}, {0xF0, 0x04}};

static esp_err_t ensure_periodic_messages(bool *changed)
{
    for (size_t i = 0; i < sizeof(PERIODIC_MESSAGES) / sizeof(PERIODIC_MESSAGES[0]); i++) {
        const uint8_t request[] = {PERIODIC_MESSAGES[i].cls, PERIODIC_MESSAGES[i].id};
        uint8_t reply[CFG_MSG_REPLY_SIZE];
        size_t length;
        ESP_RETURN_ON_ERROR(ubx_poll(CLASS_CFG, ID_CFG_MSG, request, sizeof(request), reply, sizeof(reply), &length),
                            TAG, "read message rate");
        ESP_RETURN_ON_FALSE(length == CFG_MSG_REPLY_SIZE, ESP_ERR_INVALID_SIZE, TAG, "message rate reply");
        if (reply[CFG_MSG_OFFSET_UART1_RATE] != MESSAGE_RATE) {
            const uint8_t set[CFG_MSG_SET_SIZE] = {PERIODIC_MESSAGES[i].cls, PERIODIC_MESSAGES[i].id, MESSAGE_RATE};
            ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_MSG, set, sizeof(set)), TAG, "write message rate");
            *changed = true;
        }
    }
    return ESP_OK;
}

static esp_err_t ensure_time_pulse(bool *changed)
{
    uint8_t tp5[TP5_PAYLOAD_SIZE];
    ESP_RETURN_ON_ERROR(poll_tp5(tp5), TAG, "read TP5");
    const uint32_t flags = get_u32(tp5 + TP5_OFFSET_FLAGS);
    if ((flags & TP5_FLAGS_MASK) == TP5_FLAGS_WANTED && get_u32(tp5 + TP5_OFFSET_PERIOD) == TP5_PERIOD_US
        && get_u32(tp5 + TP5_OFFSET_PERIOD_LOCKED) == TP5_PERIOD_US
        && get_u32(tp5 + TP5_OFFSET_LENGTH_LOCKED) == TP5_PULSE_LENGTH_US) {
        return ESP_OK;
    }
    put_u32(tp5 + TP5_OFFSET_PERIOD, TP5_PERIOD_US);
    put_u32(tp5 + TP5_OFFSET_PERIOD_LOCKED, TP5_PERIOD_US);
    put_u32(tp5 + TP5_OFFSET_LENGTH_LOCKED, TP5_PULSE_LENGTH_US);
    put_u32(tp5 + TP5_OFFSET_FLAGS, (flags & ~(uint32_t)TP5_FLAGS_MASK) | TP5_FLAGS_WANTED);
    ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_TP5, tp5, sizeof(tp5)), TAG, "write time pulse");
    *changed = true;
    return ESP_OK;
}

static esp_err_t ensure_solution_rate(bool *changed)
{
    uint8_t rate[RATE_PAYLOAD_SIZE];
    size_t length;
    ESP_RETURN_ON_ERROR(ubx_poll(CLASS_CFG, ID_CFG_RATE, NULL, 0, rate, sizeof(rate), &length), TAG, "read rate");
    ESP_RETURN_ON_FALSE(length == RATE_PAYLOAD_SIZE, ESP_ERR_INVALID_SIZE, TAG, "rate reply");
    if (get_u16(rate) != RATE_MEASUREMENT_MS || get_u16(rate + 2) != RATE_SOLUTIONS_PER_MEASUREMENT) {
        put_u16(rate, RATE_MEASUREMENT_MS);
        put_u16(rate + 2, RATE_SOLUTIONS_PER_MEASUREMENT);
        ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_RATE, rate, sizeof(rate)), TAG, "write rate");
        *changed = true;
    }
    return ESP_OK;
}

// The sentence parser knows the numbering and talker IDs of NMEA 4.0 only.
static esp_err_t ensure_nmea_version(bool *changed)
{
    uint8_t nmea[NMEA_MAX_PAYLOAD_SIZE];
    size_t length;
    ESP_RETURN_ON_ERROR(ubx_poll(CLASS_CFG, ID_CFG_NMEA, NULL, 0, nmea, sizeof(nmea), &length), TAG, "read NMEA");
    ESP_RETURN_ON_FALSE(length > NMEA_VERSION_OFFSET, ESP_ERR_INVALID_SIZE, TAG, "NMEA reply");
    if (nmea[NMEA_VERSION_OFFSET] != NMEA_VERSION_4_0) {
        nmea[NMEA_VERSION_OFFSET] = NMEA_VERSION_4_0;
        ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_NMEA, nmea, length), TAG, "write NMEA");
        *changed = true;
    }
    return ESP_OK;
}

static esp_err_t ensure_delays(const pps_delay_settings_t *wanted, bool *changed)
{
    uint8_t tp5[TP5_PAYLOAD_SIZE];
    ESP_RETURN_ON_ERROR(poll_tp5(tp5), TAG, "read TP5");
    if (!wanted->configured) {
        return ESP_OK;
    }
    const int16_t cable = (int16_t)get_u16(tp5 + TP5_OFFSET_CABLE_DELAY);
    const int32_t user = (int32_t)get_u32(tp5 + TP5_OFFSET_USER_DELAY);
    if (cable != wanted->cable_delay_ns || user != wanted->user_delay_ns) {
        put_u16(tp5 + TP5_OFFSET_CABLE_DELAY, (uint16_t)wanted->cable_delay_ns);
        put_u32(tp5 + TP5_OFFSET_USER_DELAY, (uint32_t)wanted->user_delay_ns);
        ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_TP5, tp5, sizeof(tp5)), TAG, "write TP5");
        *changed = true;
    }
    return ESP_OK;
}

// Galileo is off in the receiver's defaults. Changing the constellations restarts the receiver's GNSS subsystem.
static esp_err_t ensure_galileo(bool *changed)
{
    uint8_t gnss[UBX_MAX_PAYLOAD];
    size_t length;
    ESP_RETURN_ON_ERROR(ubx_poll(CLASS_CFG, ID_CFG_GNSS, NULL, 0, gnss, sizeof(gnss), &length), TAG, "read GNSS");
    ESP_RETURN_ON_FALSE(length >= GNSS_HEADER_SIZE
                            && length == GNSS_HEADER_SIZE + (size_t)gnss[GNSS_OFFSET_BLOCK_COUNT] * GNSS_BLOCK_SIZE,
                        ESP_ERR_INVALID_SIZE, TAG, "GNSS reply");
    for (size_t offset = GNSS_HEADER_SIZE; offset < length; offset += GNSS_BLOCK_SIZE) {
        if (gnss[offset + GNSS_BLOCK_OFFSET_ID] != GNSS_ID_GALILEO) {
            continue;
        }
        uint32_t flags = get_u32(gnss + offset + GNSS_BLOCK_OFFSET_FLAGS);
        if (flags & GNSS_FLAG_ENABLE) {
            return ESP_OK;
        }
        if (!(flags & GNSS_SIGNALS_MASK)) {
            flags |= GNSS_SIGNAL_GALILEO_E1;
        }
        put_u32(gnss + offset + GNSS_BLOCK_OFFSET_FLAGS, flags | GNSS_FLAG_ENABLE);
        ESP_RETURN_ON_ERROR(ubx_set(CLASS_CFG, ID_CFG_GNSS, gnss, length), TAG, "write GNSS");
        vTaskDelay(pdMS_TO_TICKS(GNSS_RESTART_MS));
        *changed = true;
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

// Persisting costs flash cycles in the receiver, so it only happens after an actual change.
static esp_err_t save_to_receiver(void)
{
    uint8_t payload[CFG_PAYLOAD_SIZE] = {0};
    put_u32(payload + 4, CFG_ALL_SECTIONS);
    payload[12] = CFG_DEVICE_BBR_AND_FLASH;
    return ubx_set(CLASS_CFG, ID_CFG_CFG, payload, sizeof(payload));
}

static esp_err_t refresh_state(const pps_delay_settings_t *wanted, bool saved)
{
    uint8_t nav5[NAV5_PAYLOAD_SIZE], tp5[TP5_PAYLOAD_SIZE];
    ESP_RETURN_ON_ERROR(poll_nav5(nav5), TAG, "re-read NAV5");
    ESP_RETURN_ON_ERROR(poll_tp5(tp5), TAG, "re-read TP5");
    portENTER_CRITICAL(&receiver_lock);
    receiver.known = true;
    receiver.saved = receiver.saved || saved;
    receiver.dynamic_model = nav5[NAV5_OFFSET_DYNMODEL];
    receiver.cable_delay_ns = (int16_t)get_u16(tp5 + TP5_OFFSET_CABLE_DELAY);
    receiver.user_delay_ns = (int32_t)get_u32(tp5 + TP5_OFFSET_USER_DELAY);
    receiver.delays_managed = wanted->configured;
    portEXIT_CRITICAL(&receiver_lock);
    ESP_LOGI(TAG, "dynamic model %u, cable delay %d ns, user delay %ld ns", nav5[NAV5_OFFSET_DYNMODEL],
             (int16_t)get_u16(tp5 + TP5_OFFSET_CABLE_DELAY), (long)(int32_t)get_u32(tp5 + TP5_OFFSET_USER_DELAY));
    return ESP_OK;
}

// Speeds a receiver may be left at, factory 9600 first. The one it answers at is changed to GNSS_BAUD and kept.
static const uint32_t PROBE_BAUDS[] = {9600, 38400, 57600, 19200, 4800, 230400, 460800};

// Returns ESP_OK when the receiver already answers at GNSS_BAUD or has been moved there.
static esp_err_t ensure_baud(void)
{
    const uint8_t request[] = {PRT_PORT_UART1};
    uint8_t port[PRT_PAYLOAD_SIZE];
    size_t length;
    if (ubx_poll(CLASS_CFG, ID_CFG_PRT, request, sizeof(request), port, sizeof(port), &length) == ESP_OK) {
        return ESP_OK;
    }
    for (size_t i = 0; i < sizeof(PROBE_BAUDS) / sizeof(PROBE_BAUDS[0]); i++) {
        ESP_RETURN_ON_ERROR(gnss_set_baud(PROBE_BAUDS[i]), TAG, "probe baud");
        esp_err_t err = ubx_poll(CLASS_CFG, ID_CFG_PRT, request, sizeof(request), port, sizeof(port), &length);
        if (err == ESP_OK && length == PRT_PAYLOAD_SIZE) {
            put_u32(port + PRT_OFFSET_BAUD_RATE, GNSS_BAUD);
            err = ubx_set(CLASS_CFG, ID_CFG_PRT, port, sizeof(port));
            ESP_RETURN_ON_ERROR(gnss_set_baud(GNSS_BAUD), TAG, "restore baud");
            ESP_RETURN_ON_ERROR(err, TAG, "move receiver to %d baud", GNSS_BAUD);
            vTaskDelay(pdMS_TO_TICKS(PORT_RECONFIGURE_MS));
            ESP_RETURN_ON_ERROR(save_to_receiver(), TAG, "save baud");
            ESP_LOGW(TAG, "receiver moved from %lu to %d baud", (unsigned long)PROBE_BAUDS[i], GNSS_BAUD);
            return ESP_OK;
        }
    }
    gnss_set_baud(GNSS_BAUD);
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t apply(bool *written)
{
    pps_delay_settings_t wanted;
    ESP_RETURN_ON_ERROR(settings_load_pps_delay(&wanted), TAG, "load delays");

    bool changed = false;
    ESP_RETURN_ON_ERROR(ensure_stationary(&changed), TAG, "dynamic model");
    ESP_RETURN_ON_ERROR(ensure_time_pulse(&changed), TAG, "time pulse");
    ESP_RETURN_ON_ERROR(ensure_delays(&wanted, &changed), TAG, "delays");
    ESP_RETURN_ON_ERROR(ensure_solution_rate(&changed), TAG, "solution rate");
    ESP_RETURN_ON_ERROR(ensure_nmea_version(&changed), TAG, "NMEA version");
    ESP_RETURN_ON_ERROR(ensure_ubx_output(&changed), TAG, "ubx output");
    ESP_RETURN_ON_ERROR(ensure_periodic_messages(&changed), TAG, "periodic messages");
    ESP_RETURN_ON_ERROR(ensure_interference_monitor(&changed), TAG, "interference monitor");
    ESP_RETURN_ON_ERROR(ensure_galileo(&changed), TAG, "Galileo");

    bool saved = false;
    *written = changed;
    if (changed) {
        ESP_RETURN_ON_ERROR(save_to_receiver(), TAG, "save to receiver memory");
        saved = true;
        ESP_LOGI(TAG, "receiver configuration changed and saved");
    } else {
        ESP_LOGI(TAG, "receiver already configured");
    }
    return refresh_state(&wanted, saved);
}

esp_err_t gnss_config_set_pps_delay(int16_t cable_delay_ns, int32_t user_delay_ns, bool *written)
{
    ESP_RETURN_ON_FALSE(cable_delay_ns >= 0 && user_delay_ns >= -GNSS_DELAY_USER_LIMIT_NS
                            && user_delay_ns <= GNSS_DELAY_USER_LIMIT_NS,
                        ESP_ERR_INVALID_ARG, TAG, "delay out of range");
    const pps_delay_settings_t wanted = {true, cable_delay_ns, user_delay_ns};
    ESP_RETURN_ON_ERROR(settings_save_pps_delay(&wanted), TAG, "store delays");
    return apply(written);
}

void gnss_config_get(gnss_receiver_t *out)
{
    portENTER_CRITICAL(&receiver_lock);
    *out = receiver;
    portEXIT_CRITICAL(&receiver_lock);
}

static void config_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));
    bool written;
    esp_err_t err = ensure_baud();
    if (err == ESP_OK) {
        err = apply(&written);
    }
    if (err == ESP_OK) {
        ubx_request(CLASS_MON, ID_MON_VER);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "receiver not configured: %s (UBX input may be disabled on its UART)", esp_err_to_name(err));
    }
    vTaskDelete(NULL);
}

esp_err_t gnss_config_start(void)
{
    ESP_RETURN_ON_FALSE(xTaskCreate(config_task, "gnss_config", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

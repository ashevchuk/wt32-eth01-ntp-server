#include "gnss.h"

#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "ubx.h"

#define GNSS_UART UART_NUM_1
#define GNSS_TX_GPIO GPIO_NUM_17
#define GNSS_RX_GPIO GPIO_NUM_5
#define GNSS_PPS_GPIO GPIO_NUM_35
#define GNSS_BAUD 115200
#define UART_RX_BUFFER_SIZE 2048
#define NMEA_MAX_LINE 128
#define NMEA_MAX_FIELDS 24
#define PPS_QUEUE_LENGTH 4
#define RMC_FIELD_TIME 1
#define RMC_FIELD_STATUS 2
#define RMC_FIELD_SPEED 7
#define RMC_FIELD_COURSE 8
#define RMC_FIELD_DATE 9
#define GGA_FIELD_LATITUDE 2
#define GGA_FIELD_LATITUDE_HEMISPHERE 3
#define GGA_FIELD_LONGITUDE 4
#define GGA_FIELD_LONGITUDE_HEMISPHERE 5
#define GGA_FIELD_QUALITY 6
#define GGA_FIELD_SATELLITES 7
#define GGA_FIELD_HDOP 8
#define GGA_FIELD_ALTITUDE 9
#define GSA_FIELD_FIX_MODE 2
#define GSA_FIELD_PDOP 15
#define GSV_FIELD_IN_VIEW 3
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_MON 0x0A
#define UBX_ID_NAV_PVT 0x07
#define UBX_ID_NAV_SAT 0x35
#define UBX_ID_MON_HW 0x09
#define NAV_SAT_HEADER_SIZE 8
#define NAV_SAT_OFFSET_COUNT 5
#define NAV_SAT_ENTRY_SIZE 12
#define SAT_FLAGS_QUALITY_MASK 0x07
#define SAT_FLAG_USED (1u << 3)
#define SAT_FLAG_EPHEMERIS (1u << 11)
#define SAT_FLAG_ALMANAC (1u << 12)
#define SAT_ELEVATION_MIN_VALID (-90)
#define PVT_PAYLOAD_SIZE 92
#define PVT_OFFSET_VALID 11
#define PVT_OFFSET_TIME_ACCURACY 12
#define PVT_OFFSET_FIX_TYPE 20
#define PVT_OFFSET_FLAGS 21
#define PVT_OFFSET_SATELLITES 23
#define PVT_OFFSET_HORIZONTAL_ACCURACY 40
#define PVT_VALID_TIME 0x02
#define PVT_VALID_RESOLVED 0x04
#define PVT_FLAG_FIX_OK 0x01
#define ACCURACY_UNKNOWN 0xFFFFFFFFu
// Looser than any real fix: a larger estimate means the receiver does not know the time well.
#define MAX_TIME_ACCURACY_NS 1000000
#define HW_PAYLOAD_SIZE 60
#define HW_OFFSET_NOISE 16
#define HW_OFFSET_AGC 18
#define HW_OFFSET_ANTENNA_STATUS 20
#define HW_OFFSET_FLAGS 22
#define HW_OFFSET_JAMMING_INDICATOR 45
#define HW_JAMMING_STATE_SHIFT 2
#define HW_JAMMING_STATE_MASK 0x03
#define HW_AGC_MAX 8191.0f
#define UBX_MESSAGE_STALE_US 3000000
#define PPS_WINDOW 60
#define PPS_NOMINAL_PERIOD_US 1000000
// A larger deviation is a missed pulse or a restart, not jitter.
#define PPS_MAX_PERIOD_ERROR_US 10000
#define PPS_PRESENT_TIMEOUT_US 2500000
#define DEGREE_MINUTES_DIVISOR 100.0
#define MINUTES_PER_DEGREE 60.0
#define KNOTS_TO_KMH 1.852f
#define TALKER_LENGTH 2
#define SENTENCE_ID_LENGTH 5
#define RMC_TIME_DIGITS 6
#define RMC_DATE_DIGITS 6
#define RMC_MAX_DELAY_US 500000
// An inserted leap second is reported as 23:59:60 and is followed directly by 00:00:00 of the next day, which
// timegm() already yields for second 60, so no second is added then.
#define LEAP_SECOND_FIELD 60
#define YEAR_BASE_TM 100

static const char *TAG = "gnss";

struct rmc_sample {
    struct tm utc;
    int64_t received_us;
};

static const char *const CONSTELLATION_TALKERS[] = {"GP", "GL", "GA", "GB", "BD", "GQ"};
#define CONSTELLATION_COUNT (sizeof(CONSTELLATION_TALKERS) / sizeof(CONSTELLATION_TALKERS[0]))

static gnss_status_t working_status;
static gnss_status_t published_status;
static uint8_t in_view_by_constellation[CONSTELLATION_COUNT];
static portMUX_TYPE status_lock = portMUX_INITIALIZER_UNLOCKED;

static gnss_satellite_t published_satellites[GNSS_MAX_SATELLITES];
static size_t published_satellite_count;
static int64_t satellites_updated_us;
static gnss_pvt_t pvt_state;
static int64_t pvt_updated_us;
static gnss_hardware_t hardware_state;
static int64_t hardware_updated_us;

static gnss_pps_t pps_stats;
static int64_t last_pps_us;

static QueueHandle_t pps_queue;
static QueueHandle_t rmc_queue;
static gnss_second_cb_t second_callback;

static void IRAM_ATTR on_pps(void *arg)
{
    int64_t timestamp_us = esp_timer_get_time();
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(pps_queue, &timestamp_us, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

static void record_pps_period(int64_t period_us)
{
    static int32_t errors[PPS_WINDOW];
    static size_t filled, next;

    const int64_t error_us = period_us - PPS_NOMINAL_PERIOD_US;
    if (llabs(error_us) > PPS_MAX_PERIOD_ERROR_US) {
        return;
    }
    errors[next] = (int32_t)error_us;
    next = (next + 1) % PPS_WINDOW;
    if (filled < PPS_WINDOW) {
        filled++;
    }

    double sum = 0, sum_squares = 0;
    int32_t min_us = errors[0], max_us = errors[0];
    for (size_t i = 0; i < filled; i++) {
        sum += errors[i];
        sum_squares += (double)errors[i] * errors[i];
        min_us = errors[i] < min_us ? errors[i] : min_us;
        max_us = errors[i] > max_us ? errors[i] : max_us;
    }
    const double mean = sum / filled;
    const double variance = sum_squares / filled - mean * mean;

    portENTER_CRITICAL(&status_lock);
    pps_stats.samples = (uint8_t)filled;
    pps_stats.offset_ppm = (float)mean;
    pps_stats.jitter_us = variance > 0 ? (float)sqrt(variance) : 0;
    pps_stats.min_error_us = min_us;
    pps_stats.max_error_us = max_us;
    portEXIT_CRITICAL(&status_lock);
}

static void pps_task(void *arg)
{
    int64_t previous_us = 0;
    int64_t timestamp_us;
    while (xQueueReceive(pps_queue, &timestamp_us, portMAX_DELAY)) {
        struct rmc_sample sample;
        portENTER_CRITICAL(&status_lock);
        pps_stats.pulses++;
        last_pps_us = timestamp_us;
        portEXIT_CRITICAL(&status_lock);
        if (previous_us) {
            ESP_LOGI(TAG, "pps period %lld us", (long long)(timestamp_us - previous_us));
            record_pps_period(timestamp_us - previous_us);
        }
        // The RMC sentence describes the second that began at the previous pulse, so this pulse starts the next one.
        bool fresh = previous_us && xQueueReceive(rmc_queue, &sample, 0) == pdTRUE
                     && sample.received_us >= previous_us
                     && sample.received_us - previous_us < RMC_MAX_DELAY_US;
        gnss_pvt_t pvt;
        gnss_get_pvt(&pvt);
        static bool rejection_logged;
        if (fresh && !pvt.time_trusted != rejection_logged) {
            rejection_logged = !pvt.time_trusted;
            ESP_LOGW(TAG, "GNSS time %s: valid %d resolved %d accuracy %lld ns", rejection_logged ? "rejected" : "accepted again",
                     pvt.time_valid, pvt.time_resolved, (long long)pvt.time_accuracy_ns);
        }
        if (fresh && second_callback && pvt.time_trusted) {
            if (sample.utc.tm_sec != LEAP_SECOND_FIELD) {
                sample.utc.tm_sec += 1;
            }
            timegm(&sample.utc);
            second_callback(&sample.utc, timestamp_us);
        }
        previous_us = timestamp_us;
    }
}

static bool nmea_checksum_ok(const char *line, const char *star)
{
    uint8_t sum = 0;
    for (const char *p = line + 1; p < star; p++) {
        sum ^= (uint8_t)*p;
    }
    return sum == (uint8_t)strtoul(star + 1, NULL, 16);
}

static int split_fields(char *body, char *fields[], int max_fields)
{
    int count = 0;
    fields[count++] = body;
    for (char *p = body; *p && count < max_fields; p++) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

static int two_digits(const char *text)
{
    return (text[0] - '0') * 10 + (text[1] - '0');
}

static bool parse_rmc_utc(const char *time_field, const char *date_field, struct tm *utc)
{
    if (strlen(time_field) < RMC_TIME_DIGITS || strlen(date_field) < RMC_DATE_DIGITS) {
        return false;
    }
    *utc = (struct tm){
        .tm_hour = two_digits(time_field),
        .tm_min = two_digits(time_field + 2),
        .tm_sec = two_digits(time_field + 4),
        .tm_mday = two_digits(date_field),
        .tm_mon = two_digits(date_field + 2) - 1,
        .tm_year = two_digits(date_field + 4) + YEAR_BASE_TM,
    };
    return true;
}

static bool has_value(const char *field)
{
    return field[0] != '\0';
}

static double parse_coordinate(const char *value, char hemisphere)
{
    const double raw = strtod(value, NULL);
    const double degrees = (int)(raw / DEGREE_MINUTES_DIVISOR);
    const double minutes = raw - degrees * DEGREE_MINUTES_DIVISOR;
    const double result = degrees + minutes / MINUTES_PER_DEGREE;
    return (hemisphere == 'S' || hemisphere == 'W') ? -result : result;
}

static void handle_rmc(char *fields[], int count)
{
    if (count <= RMC_FIELD_DATE) {
        return;
    }
    const bool valid = fields[RMC_FIELD_STATUS][0] == 'A';
    ESP_LOGI(TAG, "rmc fix=%s time=%s date=%s", valid ? "yes" : "no",
             fields[RMC_FIELD_TIME], fields[RMC_FIELD_DATE]);

    working_status.fix = valid;
    working_status.updated_us = esp_timer_get_time();
    working_status.speed_kmh = strtof(fields[RMC_FIELD_SPEED], NULL) * KNOTS_TO_KMH;
    working_status.course_deg = strtof(fields[RMC_FIELD_COURSE], NULL);

    struct rmc_sample sample = {.received_us = working_status.updated_us};
    if (valid && parse_rmc_utc(fields[RMC_FIELD_TIME], fields[RMC_FIELD_DATE], &sample.utc)) {
        xQueueOverwrite(rmc_queue, &sample);
    } else {
        xQueueReset(rmc_queue);
    }
}

static void handle_gga(char *fields[], int count)
{
    if (count <= GGA_FIELD_ALTITUDE) {
        return;
    }
    working_status.fix_quality = (uint8_t)strtoul(fields[GGA_FIELD_QUALITY], NULL, 10);
    working_status.satellites_used = (uint8_t)strtoul(fields[GGA_FIELD_SATELLITES], NULL, 10);
    working_status.hdop = strtof(fields[GGA_FIELD_HDOP], NULL);
    working_status.has_position = working_status.fix_quality > 0 && has_value(fields[GGA_FIELD_LATITUDE])
                                  && has_value(fields[GGA_FIELD_LONGITUDE]);
    if (working_status.has_position) {
        working_status.latitude_deg = parse_coordinate(fields[GGA_FIELD_LATITUDE], fields[GGA_FIELD_LATITUDE_HEMISPHERE][0]);
        working_status.longitude_deg = parse_coordinate(fields[GGA_FIELD_LONGITUDE], fields[GGA_FIELD_LONGITUDE_HEMISPHERE][0]);
        working_status.altitude_m = strtof(fields[GGA_FIELD_ALTITUDE], NULL);
    }
}

static void handle_gsa(char *fields[], int count)
{
    if (count <= GSA_FIELD_PDOP) {
        return;
    }
    working_status.fix_mode = (uint8_t)strtoul(fields[GSA_FIELD_FIX_MODE], NULL, 10);
    working_status.pdop = strtof(fields[GSA_FIELD_PDOP], NULL);
}

static void handle_gsv(const char *talker, char *fields[], int count)
{
    if (count <= GSV_FIELD_IN_VIEW) {
        return;
    }
    uint32_t total = 0;
    for (size_t i = 0; i < CONSTELLATION_COUNT; i++) {
        if (strncmp(talker, CONSTELLATION_TALKERS[i], TALKER_LENGTH) == 0) {
            in_view_by_constellation[i] = (uint8_t)strtoul(fields[GSV_FIELD_IN_VIEW], NULL, 10);
        }
        total += in_view_by_constellation[i];
    }
    working_status.satellites_in_view = (uint8_t)total;
}

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void handle_nav_sat(const uint8_t *payload, size_t length)
{
    static gnss_satellite_t parsed[GNSS_MAX_SATELLITES];
    if (length < NAV_SAT_HEADER_SIZE) {
        return;
    }
    size_t count = payload[NAV_SAT_OFFSET_COUNT];
    count = count > (length - NAV_SAT_HEADER_SIZE) / NAV_SAT_ENTRY_SIZE ? (length - NAV_SAT_HEADER_SIZE) / NAV_SAT_ENTRY_SIZE : count;
    count = count > GNSS_MAX_SATELLITES ? GNSS_MAX_SATELLITES : count;
    for (size_t i = 0; i < count; i++) {
        const uint8_t *entry = payload + NAV_SAT_HEADER_SIZE + i * NAV_SAT_ENTRY_SIZE;
        const int elevation = (int8_t)entry[3];
        const uint32_t flags = read_u32(entry + 8);
        const bool has_direction = elevation >= SAT_ELEVATION_MIN_VALID;
        parsed[i] = (gnss_satellite_t){
            .gnss_id = entry[0],
            .id = entry[1],
            .elevation_deg = has_direction ? elevation : GNSS_UNKNOWN,
            .azimuth_deg = has_direction ? (int16_t)read_u16(entry + 4) : GNSS_UNKNOWN,
            .snr_dbhz = entry[2] ? entry[2] : GNSS_UNKNOWN,
            .quality = flags & SAT_FLAGS_QUALITY_MASK,
            .used = flags & SAT_FLAG_USED,
            .ephemeris = flags & SAT_FLAG_EPHEMERIS,
            .almanac = flags & SAT_FLAG_ALMANAC,
        };
    }
    portENTER_CRITICAL(&status_lock);
    memcpy(published_satellites, parsed, count * sizeof(gnss_satellite_t));
    published_satellite_count = count;
    satellites_updated_us = esp_timer_get_time();
    portEXIT_CRITICAL(&status_lock);
}

static void handle_nav_pvt(const uint8_t *payload, size_t length)
{
    if (length < PVT_PAYLOAD_SIZE) {
        return;
    }
    const uint32_t time_accuracy = read_u32(payload + PVT_OFFSET_TIME_ACCURACY);
    const uint32_t horizontal_accuracy = read_u32(payload + PVT_OFFSET_HORIZONTAL_ACCURACY);
    const gnss_pvt_t pvt = {
        .fix_type = payload[PVT_OFFSET_FIX_TYPE],
        .fix_ok = payload[PVT_OFFSET_FLAGS] & PVT_FLAG_FIX_OK,
        .time_valid = payload[PVT_OFFSET_VALID] & PVT_VALID_TIME,
        .time_resolved = payload[PVT_OFFSET_VALID] & PVT_VALID_RESOLVED,
        .satellites = payload[PVT_OFFSET_SATELLITES],
        .time_accuracy_ns = time_accuracy == ACCURACY_UNKNOWN ? -1 : (int64_t)time_accuracy,
        .horizontal_accuracy_m = horizontal_accuracy == ACCURACY_UNKNOWN ? -1.0f : horizontal_accuracy / 1000.0f,
    };
    portENTER_CRITICAL(&status_lock);
    pvt_state = pvt;
    pvt_updated_us = esp_timer_get_time();
    portEXIT_CRITICAL(&status_lock);
}

static void handle_mon_hw(const uint8_t *payload, size_t length)
{
    if (length < HW_PAYLOAD_SIZE) {
        return;
    }
    const gnss_hardware_t hardware = {
        .antenna_status = payload[HW_OFFSET_ANTENNA_STATUS],
        .jamming_state = (payload[HW_OFFSET_FLAGS] >> HW_JAMMING_STATE_SHIFT) & HW_JAMMING_STATE_MASK,
        .jamming_indicator = payload[HW_OFFSET_JAMMING_INDICATOR],
        .agc_percent = read_u16(payload + HW_OFFSET_AGC) * 100.0f / HW_AGC_MAX,
        .noise_per_ms = read_u16(payload + HW_OFFSET_NOISE),
    };
    portENTER_CRITICAL(&status_lock);
    hardware_state = hardware;
    hardware_updated_us = esp_timer_get_time();
    portEXIT_CRITICAL(&status_lock);
}

static void on_ubx_message(uint8_t cls, uint8_t id, const uint8_t *payload, size_t length)
{
    if (cls == UBX_CLASS_NAV && id == UBX_ID_NAV_SAT) {
        handle_nav_sat(payload, length);
    } else if (cls == UBX_CLASS_NAV && id == UBX_ID_NAV_PVT) {
        handle_nav_pvt(payload, length);
    } else if (cls == UBX_CLASS_MON && id == UBX_ID_MON_HW) {
        handle_mon_hw(payload, length);
    }
}

static void handle_nmea_line(char *line)
{
    char *star = strchr(line, '*');
    if (line[0] != '$' || !star || !nmea_checksum_ok(line, star)) {
        return;
    }
    *star = '\0';

    char *fields[NMEA_MAX_FIELDS];
    int count = split_fields(line + 1, fields, NMEA_MAX_FIELDS);
    if (strlen(fields[0]) != SENTENCE_ID_LENGTH) {
        return;
    }
    const char *type = fields[0] + TALKER_LENGTH;
    if (strcmp(type, "RMC") == 0) {
        handle_rmc(fields, count);
    } else if (strcmp(type, "GGA") == 0) {
        handle_gga(fields, count);
    } else if (strcmp(type, "GSA") == 0) {
        handle_gsa(fields, count);
    } else if (strcmp(type, "GSV") == 0) {
        handle_gsv(fields[0], fields, count);
    } else {
        return;
    }

    portENTER_CRITICAL(&status_lock);
    published_status = working_status;
    portEXIT_CRITICAL(&status_lock);
}

void gnss_get_status(gnss_status_t *status)
{
    portENTER_CRITICAL(&status_lock);
    *status = published_status;
    portEXIT_CRITICAL(&status_lock);
}

size_t gnss_get_satellites(gnss_satellite_t satellites[GNSS_MAX_SATELLITES], int64_t *age_us)
{
    portENTER_CRITICAL(&status_lock);
    const size_t count = published_satellite_count;
    memcpy(satellites, published_satellites, count * sizeof(gnss_satellite_t));
    *age_us = satellites_updated_us ? esp_timer_get_time() - satellites_updated_us : INT64_MAX;
    portEXIT_CRITICAL(&status_lock);
    return count;
}

// RMC alone decides while no NAV-PVT arrives. Otherwise the receiver must report the time as valid and resolved, and
// its accuracy estimate, when it has one, must be within the limit.
static bool pvt_allows_time(const gnss_pvt_t *pvt)
{
    if (!pvt->known) {
        return true;
    }
    if (!pvt->time_valid || !pvt->time_resolved) {
        return false;
    }
    return pvt->time_accuracy_ns < 0 || pvt->time_accuracy_ns <= MAX_TIME_ACCURACY_NS;
}

void gnss_get_pvt(gnss_pvt_t *pvt)
{
    portENTER_CRITICAL(&status_lock);
    *pvt = pvt_state;
    pvt->known = pvt_updated_us && esp_timer_get_time() - pvt_updated_us < UBX_MESSAGE_STALE_US;
    portEXIT_CRITICAL(&status_lock);
    pvt->time_trusted = pvt_allows_time(pvt);
}

void gnss_get_hardware(gnss_hardware_t *hardware)
{
    portENTER_CRITICAL(&status_lock);
    *hardware = hardware_state;
    hardware->known = hardware_updated_us && esp_timer_get_time() - hardware_updated_us < UBX_MESSAGE_STALE_US;
    portEXIT_CRITICAL(&status_lock);
}

void gnss_get_pps(gnss_pps_t *pps)
{
    portENTER_CRITICAL(&status_lock);
    *pps = pps_stats;
    pps->present = last_pps_us && esp_timer_get_time() - last_pps_us < PPS_PRESENT_TIMEOUT_US;
    portEXIT_CRITICAL(&status_lock);
}

static void nmea_task(void *arg)
{
    char line[NMEA_MAX_LINE];
    size_t length = 0;
    uint8_t byte;
    while (true) {
        if (uart_read_bytes(GNSS_UART, &byte, 1, portMAX_DELAY) != 1) {
            continue;
        }
        if (ubx_feed(byte)) {
            continue;
        }
        if (byte == '\n') {
            line[length] = '\0';
            handle_nmea_line(line);
            length = 0;
        } else if (byte != '\r' && length < sizeof(line) - 1) {
            line[length++] = (char)byte;
        } else if (length >= sizeof(line) - 1) {
            length = 0;
        }
    }
}

static esp_err_t start_uart(void)
{
    const uart_config_t cfg = {
        .baud_rate = GNSS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(GNSS_UART, UART_RX_BUFFER_SIZE, 0, 0, NULL, 0), TAG, "uart install");
    ESP_RETURN_ON_ERROR(uart_param_config(GNSS_UART, &cfg), TAG, "uart config");
    return uart_set_pin(GNSS_UART, GNSS_TX_GPIO, GNSS_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static esp_err_t start_pps(void)
{
    pps_queue = xQueueCreate(PPS_QUEUE_LENGTH, sizeof(int64_t));
    rmc_queue = xQueueCreate(1, sizeof(struct rmc_sample));
    ESP_RETURN_ON_FALSE(pps_queue && rmc_queue, ESP_ERR_NO_MEM, TAG, "queues");

    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << GNSS_PPS_GPIO,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "pps gpio config");
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "isr service");
    return gpio_isr_handler_add(GNSS_PPS_GPIO, on_pps, NULL);
}

esp_err_t gnss_start(gnss_second_cb_t on_second)
{
    second_callback = on_second;
    ESP_RETURN_ON_ERROR(ubx_init(GNSS_UART), TAG, "ubx");
    ubx_set_handler(on_ubx_message);
    ESP_RETURN_ON_ERROR(start_uart(), TAG, "uart");
    ESP_RETURN_ON_ERROR(start_pps(), TAG, "pps");
    ESP_RETURN_ON_FALSE(xTaskCreate(nmea_task, "nmea", 4096, NULL, 5, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "nmea task");
    ESP_RETURN_ON_FALSE(xTaskCreate(pps_task, "pps", 3072, NULL, 5, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "pps task");
    return ESP_OK;
}

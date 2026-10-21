#include "gnss.h"

#include <stdbool.h>
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
#define UART_RX_BUFFER_SIZE 1024
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

static void pps_task(void *arg)
{
    int64_t previous_us = 0;
    int64_t timestamp_us;
    while (xQueueReceive(pps_queue, &timestamp_us, portMAX_DELAY)) {
        struct rmc_sample sample;
        if (previous_us) {
            ESP_LOGI(TAG, "pps period %lld us", (long long)(timestamp_us - previous_us));
        }
        // The RMC sentence describes the second that began at the previous pulse, so this pulse starts the next one.
        bool fresh = previous_us && xQueueReceive(rmc_queue, &sample, 0) == pdTRUE
                     && sample.received_us >= previous_us
                     && sample.received_us - previous_us < RMC_MAX_DELAY_US;
        if (fresh && second_callback) {
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
    ESP_RETURN_ON_ERROR(start_uart(), TAG, "uart");
    ESP_RETURN_ON_ERROR(start_pps(), TAG, "pps");
    ESP_RETURN_ON_FALSE(xTaskCreate(nmea_task, "nmea", 4096, NULL, 5, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "nmea task");
    ESP_RETURN_ON_FALSE(xTaskCreate(pps_task, "pps", 3072, NULL, 5, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "pps task");
    return ESP_OK;
}

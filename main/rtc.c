#include "rtc.h"

#include <stdint.h>

#include "esp_check.h"
#include "esp_log.h"

#define DS3231_ADDRESS 0x68
#define I2C_CLOCK_HZ 100000
#define I2C_TIMEOUT_MS 100

#define REG_TIME 0x00
#define REG_STATUS 0x0F
#define TIME_REGISTER_COUNT 7
#define STATUS_OSCILLATOR_STOPPED 0x80

#define HOURS_24H_MASK 0x3F
#define MONTH_MASK 0x1F
#define YEAR_BASE_TM 100
#define WEEKDAY_OFFSET 1

static const char *TAG = "rtc";

static i2c_master_dev_handle_t rtc_dev;

static uint8_t bcd_to_bin(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

static uint8_t bin_to_bcd(int bin)
{
    return (uint8_t)(((bin / 10) << 4) | (bin % 10));
}

static esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(rtc_dev, &reg, 1, data, length, I2C_TIMEOUT_MS);
}

static esp_err_t clear_oscillator_stopped(void)
{
    uint8_t buffer[2] = {REG_STATUS};
    ESP_RETURN_ON_ERROR(read_registers(REG_STATUS, &buffer[1], 1), TAG, "read status");
    buffer[1] &= (uint8_t)~STATUS_OSCILLATOR_STOPPED;
    return i2c_master_transmit(rtc_dev, buffer, sizeof(buffer), I2C_TIMEOUT_MS);
}

esp_err_t rtc_read(struct tm *utc)
{
    if (!rtc_dev) {
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t status;
    ESP_RETURN_ON_ERROR(read_registers(REG_STATUS, &status, 1), TAG, "read status");

    uint8_t raw[TIME_REGISTER_COUNT];
    ESP_RETURN_ON_ERROR(read_registers(REG_TIME, raw, sizeof(raw)), TAG, "read time");

    *utc = (struct tm){
        .tm_sec = bcd_to_bin(raw[0]),
        .tm_min = bcd_to_bin(raw[1]),
        .tm_hour = bcd_to_bin(raw[2] & HOURS_24H_MASK),
        .tm_wday = raw[3] - WEEKDAY_OFFSET,
        .tm_mday = bcd_to_bin(raw[4]),
        .tm_mon = bcd_to_bin(raw[5] & MONTH_MASK) - 1,
        .tm_year = bcd_to_bin(raw[6]) + YEAR_BASE_TM,
    };
    return (status & STATUS_OSCILLATOR_STOPPED) ? ESP_ERR_INVALID_STATE : ESP_OK;
}

esp_err_t rtc_write(const struct tm *utc)
{
    if (!rtc_dev) {
        return ESP_ERR_NOT_FOUND;
    }
    const uint8_t buffer[1 + TIME_REGISTER_COUNT] = {
        REG_TIME,
        bin_to_bcd(utc->tm_sec),
        bin_to_bcd(utc->tm_min),
        bin_to_bcd(utc->tm_hour),
        (uint8_t)(utc->tm_wday + WEEKDAY_OFFSET),
        bin_to_bcd(utc->tm_mday),
        bin_to_bcd(utc->tm_mon + 1),
        bin_to_bcd(utc->tm_year - YEAR_BASE_TM),
    };
    ESP_RETURN_ON_ERROR(i2c_master_transmit(rtc_dev, buffer, sizeof(buffer), I2C_TIMEOUT_MS), TAG, "write time");
    return clear_oscillator_stopped();
}

esp_err_t rtc_start(i2c_master_bus_handle_t bus)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS3231_ADDRESS,
        .scl_speed_hz = I2C_CLOCK_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &rtc_dev), TAG, "add device");

    struct tm now;
    esp_err_t err = rtc_read(&now);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "oscillator was stopped, time is not valid");
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "initial read");
    ESP_LOGI(TAG, "%04d-%02d-%02d %02d:%02d:%02d", now.tm_year + 1900, now.tm_mon + 1, now.tm_mday,
             now.tm_hour, now.tm_min, now.tm_sec);
    return ESP_OK;
}

#include "lcd.h"

#include <stdint.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_CLOCK_HZ 100000
#define I2C_TIMEOUT_MS 100
#define PROBE_TIMEOUT_MS 50
#define PCF8574_ADDRESS_A 0x27
#define PCF8574_ADDRESS_B 0x3F

// PCF8574 backpack wiring: P0=RS, P1=RW, P2=E, P3=backlight, P4..P7=D4..D7.
#define PIN_RS 0x01
#define PIN_E 0x04
#define PIN_BACKLIGHT 0x08

#define CMD_CLEAR 0x01
#define CMD_ENTRY_MODE_INCREMENT 0x06
#define CMD_DISPLAY_ON 0x0C
#define CMD_FUNCTION_SET_4BIT_2LINE 0x28
#define CMD_SET_DDRAM_ADDRESS 0x80
#define CMD_SET_CGRAM_ADDRESS 0x40
#define CUSTOM_CHAR_COUNT 8
#define CUSTOM_CHAR_ROWS 8
#define CLEAR_DELAY_US 2000
#define INIT_STEP_DELAY_US 5000
#define INIT_NIBBLE_8BIT 0x3
#define INIT_NIBBLE_4BIT 0x2
#define START_ATTEMPTS 5
#define START_RETRY_MS 300

static const char *TAG = "lcd";

static const uint8_t ROW_OFFSETS[LCD_ROWS] = {0x00, 0x40, 0x14, 0x54};

static i2c_master_dev_handle_t lcd_dev;

static esp_err_t write_expander(uint8_t value)
{
    return i2c_master_transmit(lcd_dev, &value, 1, I2C_TIMEOUT_MS);
}

static esp_err_t write_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (uint8_t)(nibble << 4) | PIN_BACKLIGHT | rs;
    ESP_RETURN_ON_ERROR(write_expander(data | PIN_E), TAG, "enable high");
    esp_rom_delay_us(1);
    ESP_RETURN_ON_ERROR(write_expander(data), TAG, "enable low");
    esp_rom_delay_us(50);
    return ESP_OK;
}

static esp_err_t write_byte(uint8_t value, uint8_t rs)
{
    ESP_RETURN_ON_ERROR(write_nibble(value >> 4, rs), TAG, "high nibble");
    return write_nibble(value & 0x0F, rs);
}

static esp_err_t write_command(uint8_t command)
{
    return write_byte(command, 0);
}

static esp_err_t find_address(i2c_master_bus_handle_t bus, uint16_t *address)
{
    const uint16_t candidates[] = {PCF8574_ADDRESS_A, PCF8574_ADDRESS_B};
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        if (i2c_master_probe(bus, candidates[i], PROBE_TIMEOUT_MS) == ESP_OK) {
            *address = candidates[i];
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t init_controller(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));
    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_ERROR(write_nibble(INIT_NIBBLE_8BIT, 0), TAG, "8-bit init");
        esp_rom_delay_us(INIT_STEP_DELAY_US);
    }
    ESP_RETURN_ON_ERROR(write_nibble(INIT_NIBBLE_4BIT, 0), TAG, "switch to 4-bit");
    ESP_RETURN_ON_ERROR(write_command(CMD_FUNCTION_SET_4BIT_2LINE), TAG, "function set");
    ESP_RETURN_ON_ERROR(write_command(CMD_DISPLAY_ON), TAG, "display on");
    ESP_RETURN_ON_ERROR(write_command(CMD_CLEAR), TAG, "clear");
    esp_rom_delay_us(CLEAR_DELAY_US);
    return write_command(CMD_ENTRY_MODE_INCREMENT);
}

static esp_err_t try_start(i2c_master_bus_handle_t bus)
{
    uint16_t address;
    ESP_RETURN_ON_ERROR(find_address(bus, &address), TAG, "no PCF8574 at 0x27 or 0x3F");
    ESP_LOGI(TAG, "pcf8574 at 0x%02x", address);

    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_CLOCK_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &lcd_dev), TAG, "add device");
    const esp_err_t err = init_controller();
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(lcd_dev);
        lcd_dev = NULL;
    }
    return err;
}

// The backpack and the controller can still be coming up when the firmware gets here, and the bus can glitch.
esp_err_t lcd_start(i2c_master_bus_handle_t bus)
{
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= START_ATTEMPTS; attempt++) {
        err = try_start(bus);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "start attempt %d of %d failed: %s", attempt, START_ATTEMPTS, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(START_RETRY_MS));
    }
    return err;
}

esp_err_t lcd_print(int row, const char *text)
{
    ESP_RETURN_ON_FALSE(row >= 0 && row < LCD_ROWS, ESP_ERR_INVALID_ARG, TAG, "row out of range");
    ESP_RETURN_ON_ERROR(write_command(CMD_SET_DDRAM_ADDRESS | ROW_OFFSETS[row]), TAG, "set address");
    for (int column = 0; column < LCD_COLUMNS; column++) {
        char c = *text ? *text++ : ' ';
        ESP_RETURN_ON_ERROR(write_byte((uint8_t)c, PIN_RS), TAG, "write char");
    }
    return ESP_OK;
}

esp_err_t lcd_clear(void)
{
    ESP_RETURN_ON_ERROR(write_command(CMD_CLEAR), TAG, "clear");
    esp_rom_delay_us(CLEAR_DELAY_US);
    return ESP_OK;
}

esp_err_t lcd_write(int row, int column, const uint8_t *cells, size_t length)
{
    ESP_RETURN_ON_FALSE(row >= 0 && row < LCD_ROWS && column >= 0 && column + (int)length <= LCD_COLUMNS,
                        ESP_ERR_INVALID_ARG, TAG, "position out of range");
    ESP_RETURN_ON_ERROR(write_command(CMD_SET_DDRAM_ADDRESS | (ROW_OFFSETS[row] + column)), TAG, "set address");
    for (size_t i = 0; i < length; i++) {
        ESP_RETURN_ON_ERROR(write_byte(cells[i], PIN_RS), TAG, "write cell");
    }
    return ESP_OK;
}

esp_err_t lcd_define_char(int index, const uint8_t rows[8])
{
    ESP_RETURN_ON_FALSE(index >= 0 && index < CUSTOM_CHAR_COUNT, ESP_ERR_INVALID_ARG, TAG, "char index out of range");
    ESP_RETURN_ON_ERROR(write_command(CMD_SET_CGRAM_ADDRESS | (index * CUSTOM_CHAR_ROWS)), TAG, "set cgram address");
    for (int i = 0; i < CUSTOM_CHAR_ROWS; i++) {
        ESP_RETURN_ON_ERROR(write_byte(rows[i], PIN_RS), TAG, "write glyph row");
    }
    return ESP_OK;
}

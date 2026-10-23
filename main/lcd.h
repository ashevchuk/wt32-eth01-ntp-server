#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define LCD_COLUMNS 20
#define LCD_ROWS 4

esp_err_t lcd_start(i2c_master_bus_handle_t bus);
// Brings the display back after a failed write: initialises the controller again, attaching the device first when
// the start failed. A single attempt; the caller decides when to try again.
esp_err_t lcd_recover(void);

esp_err_t lcd_print(int row, const char *text);
esp_err_t lcd_clear(void);

// Cell values below 8 select a custom character defined with lcd_define_char.
esp_err_t lcd_write(int row, int column, const uint8_t *cells, size_t length);
esp_err_t lcd_define_char(int index, const uint8_t rows[8]);

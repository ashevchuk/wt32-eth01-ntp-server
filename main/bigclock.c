#include "bigclock.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lcd.h"
#include "rtc.h"

// Glyphs and digit layout are those of lcdproc's 4-line, 8-custom-character bignum set.
#define BLANK ' '
#define COLON_DOT_GLYPH 0
#define DIGIT_WIDTH 3
#define DIGIT_HEIGHT 4
#define DIGIT_COUNT 6
#define GLYPH_COUNT 8
#define GLYPH_ROWS 8
#define FIRST_GLYPH_INDEX 0
#define COLON_FIRST_ROW 1
#define COLON_ROW_COUNT 2
#define POLL_INTERVAL_MS 100
#define TIME_TEXT_LENGTH 7

static const char *TAG = "bigclock";

static const uint8_t DIGIT_COLUMNS[DIGIT_COUNT] = {0, 3, 7, 10, 14, 17};
static const uint8_t COLON_COLUMNS[2] = {6, 13};

static const uint8_t GLYPHS[GLYPH_COUNT][GLYPH_ROWS] = {
    {0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00},
    {0x03, 0x03, 0x03, 0x03, 0x18, 0x18, 0x18, 0x18},
    {0x1B, 0x1B, 0x1B, 0x1B, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x18},
    {0x18, 0x18, 0x18, 0x18, 0x03, 0x03, 0x03, 0x03},
    {0x00, 0x00, 0x00, 0x00, 0x1B, 0x1B, 0x1B, 0x1B},
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18},
    {0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00},
};

static const uint8_t DIGIT_CELLS[10][DIGIT_HEIGHT][DIGIT_WIDTH] = {
    {{1, 2, 3}, {6, BLANK, 6}, {6, BLANK, 6}, {7, 2, BLANK}},
    {{7, 6, BLANK}, {BLANK, 6, BLANK}, {BLANK, 6, BLANK}, {7, 2, BLANK}},
    {{1, 2, 3}, {BLANK, 5, 0}, {1, BLANK, BLANK}, {2, 2, 0}},
    {{1, 2, 3}, {BLANK, 5, 0}, {3, BLANK, 6}, {7, 2, BLANK}},
    {{BLANK, 3, 6}, {1, BLANK, 6}, {2, 2, 6}, {BLANK, BLANK, 0}},
    {{1, 2, 0}, {2, 2, 3}, {3, BLANK, 6}, {7, 2, BLANK}},
    {{1, 2, BLANK}, {6, 5, BLANK}, {6, BLANK, 6}, {7, 2, BLANK}},
    {{2, 2, 6}, {BLANK, 1, BLANK}, {BLANK, 6, BLANK}, {BLANK, 0, BLANK}},
    {{1, 2, 3}, {4, 5, 0}, {6, BLANK, 6}, {7, 2, BLANK}},
    {{1, 2, 3}, {4, 3, 6}, {BLANK, 1, BLANK}, {7, BLANK, BLANK}},
};

enum screen_state { SCREEN_BLANK, SCREEN_WAITING, SCREEN_CLOCK };

static esp_err_t draw_digit(int slot, int digit)
{
    for (int row = 0; row < DIGIT_HEIGHT; row++) {
        ESP_RETURN_ON_ERROR(lcd_write(row, DIGIT_COLUMNS[slot], DIGIT_CELLS[digit][row], DIGIT_WIDTH), TAG, "digit row");
    }
    return ESP_OK;
}

static esp_err_t draw_colons(bool visible)
{
    const uint8_t cell = visible ? COLON_DOT_GLYPH : BLANK;
    for (int colon = 0; colon < 2; colon++) {
        for (int row = COLON_FIRST_ROW; row < COLON_FIRST_ROW + COLON_ROW_COUNT; row++) {
            ESP_RETURN_ON_ERROR(lcd_write(row, COLON_COLUMNS[colon], &cell, 1), TAG, "colon");
        }
    }
    return ESP_OK;
}

static esp_err_t define_glyphs(void)
{
    for (int i = 0; i < GLYPH_COUNT; i++) {
        ESP_RETURN_ON_ERROR(lcd_define_char(FIRST_GLYPH_INDEX + i, GLYPHS[i]), TAG, "define glyph");
    }
    return ESP_OK;
}

static esp_err_t show_clock(const struct tm *time, char shown[DIGIT_COUNT], bool *colons_shown, bool force)
{
    char text[TIME_TEXT_LENGTH];
    snprintf(text, sizeof(text), "%02d%02d%02d", time->tm_hour, time->tm_min, time->tm_sec);

    for (int slot = 0; slot < DIGIT_COUNT; slot++) {
        if (force || text[slot] != shown[slot]) {
            ESP_RETURN_ON_ERROR(draw_digit(slot, text[slot] - '0'), TAG, "digit");
            shown[slot] = text[slot];
        }
    }
    const bool colons = time->tm_sec % 2 == 0;
    if (force || colons != *colons_shown) {
        ESP_RETURN_ON_ERROR(draw_colons(colons), TAG, "colons");
        *colons_shown = colons;
    }
    return ESP_OK;
}

static void clock_task(void *arg)
{
    enum screen_state state = SCREEN_BLANK;
    char shown[DIGIT_COUNT] = {0};
    bool colons_shown = false;

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_add(NULL));
    while (true) {
        esp_task_wdt_reset();
        struct tm utc;
        esp_err_t err = rtc_read(&utc);
        if (err == ESP_ERR_INVALID_STATE && state != SCREEN_WAITING) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(lcd_clear());
            ESP_ERROR_CHECK_WITHOUT_ABORT(lcd_print(1, "  Waiting for time"));
            state = SCREEN_WAITING;
        } else if (err == ESP_OK) {
            const bool force = state != SCREEN_CLOCK;
            if (force) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(lcd_clear());
            }
            const time_t seconds = timegm(&utc);
            struct tm local;
            localtime_r(&seconds, &local);
            if (show_clock(&local, shown, &colons_shown, force) == ESP_OK) {
                state = SCREEN_CLOCK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

esp_err_t bigclock_start(void)
{
    ESP_RETURN_ON_ERROR(define_glyphs(), TAG, "glyphs");
    ESP_RETURN_ON_FALSE(xTaskCreate(clock_task, "bigclock", 4096, NULL, 4, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

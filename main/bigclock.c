#include "bigclock.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lcd.h"
#include "startup.h"
#include "rtc.h"
#include "settings.h"

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
#define TEXT_LINE_COUNT 3
#define TEXT_FIRST_ROW 1
#define TEXT_BUFFER_SIZE 48
#define DATE_COLUMN 14
#define DATE_LINE_COUNT 4
#define DATE_WIDTH (LCD_COLUMNS - DATE_COLUMN)
#define DATE_LINE_SIZE 16
#define DIGITS_WITHOUT_SECONDS 4
#define COLONS_WITHOUT_SECONDS 1
#define US_PER_SECOND 1000000

static const char *TAG = "bigclock";

static display_settings_t display = {.mode = DISPLAY_MODE_BIG, .rotate_s = 10, .rotate_faces = DISPLAY_FACES_ALL};

static const char *const WEEKDAYS[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

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

// A failed display is re-initialised no more often than this.
#define RECOVER_INTERVAL_MS 2000

enum screen_state { SCREEN_BLANK, SCREEN_WAITING, SCREEN_CLOCK };
enum clock_face { FACE_BIG, FACE_BIG_DATE, FACE_TEXT, FACE_COUNT };

static esp_err_t draw_digit(int slot, int digit)
{
    for (int row = 0; row < DIGIT_HEIGHT; row++) {
        ESP_RETURN_ON_ERROR(lcd_write(row, DIGIT_COLUMNS[slot], DIGIT_CELLS[digit][row], DIGIT_WIDTH), TAG, "digit row");
    }
    return ESP_OK;
}

static esp_err_t draw_colons(bool visible, int count)
{
    const uint8_t cell = visible ? COLON_DOT_GLYPH : BLANK;
    for (int colon = 0; colon < count; colon++) {
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

static esp_err_t show_clock(const struct tm *time, char shown[DIGIT_COUNT], bool *colons_shown, bool force,
                            bool with_seconds)
{
    const int digits = with_seconds ? DIGIT_COUNT : DIGITS_WITHOUT_SECONDS;
    char text[TIME_TEXT_LENGTH];
    snprintf(text, sizeof(text), "%02d%02d%02d", time->tm_hour, time->tm_min, time->tm_sec);

    for (int slot = 0; slot < digits; slot++) {
        if (force || text[slot] != shown[slot]) {
            ESP_RETURN_ON_ERROR(draw_digit(slot, text[slot] - '0'), TAG, "digit");
            shown[slot] = text[slot];
        }
    }
    const bool colons = time->tm_sec % 2 == 0;
    if (force || colons != *colons_shown) {
        ESP_RETURN_ON_ERROR(draw_colons(colons, with_seconds ? 2 : COLONS_WITHOUT_SECONDS), TAG, "colons");
        *colons_shown = colons;
    }
    return ESP_OK;
}

static void align_right(char line[DATE_LINE_SIZE], const char *text)
{
    const size_t length = strnlen(text, DATE_WIDTH);
    memset(line, ' ', DATE_WIDTH - length);
    memcpy(line + DATE_WIDTH - length, text, length);
    line[DATE_WIDTH] = '\0';
}

static esp_err_t show_date(const struct tm *time, char shown[DATE_LINE_COUNT][DATE_LINE_SIZE], bool force)
{
    char fields[DATE_LINE_COUNT][DATE_LINE_SIZE];
    char lines[DATE_LINE_COUNT][DATE_LINE_SIZE];
    snprintf(fields[0], DATE_LINE_SIZE, "%.3s", WEEKDAYS[time->tm_wday]);
    snprintf(fields[1], DATE_LINE_SIZE, "%02d", time->tm_mday);
    snprintf(fields[2], DATE_LINE_SIZE, "%02d", time->tm_mon + 1);
    snprintf(fields[3], DATE_LINE_SIZE, "%04d", time->tm_year + 1900);

    for (int row = 0; row < DATE_LINE_COUNT; row++) {
        align_right(lines[row], fields[row]);
        if (force || strcmp(lines[row], shown[row]) != 0) {
            ESP_RETURN_ON_ERROR(lcd_write(row, DATE_COLUMN, (const uint8_t *)lines[row], DATE_WIDTH), TAG,
                                "date line");
            strcpy(shown[row], lines[row]);
        }
    }
    return ESP_OK;
}

static void center(char line[LCD_COLUMNS + 1], const char *text)
{
    const size_t length = strnlen(text, LCD_COLUMNS);
    const size_t padding = (LCD_COLUMNS - length) / 2;
    memset(line, ' ', padding);
    memcpy(line + padding, text, length);
    line[padding + length] = '\0';
}

static esp_err_t show_text(const struct tm *time, char shown[TEXT_LINE_COUNT][LCD_COLUMNS + 1], bool force)
{
    char text[TEXT_BUFFER_SIZE];
    char lines[TEXT_LINE_COUNT][LCD_COLUMNS + 1];
    snprintf(text, sizeof(text), "%02d:%02d:%02d", time->tm_hour, time->tm_min, time->tm_sec);
    center(lines[0], text);
    snprintf(text, sizeof(text), "%04d-%02d-%02d", time->tm_year + 1900, time->tm_mon + 1, time->tm_mday);
    center(lines[1], text);
    center(lines[2], WEEKDAYS[time->tm_wday]);

    for (int line = 0; line < TEXT_LINE_COUNT; line++) {
        if (force || strcmp(lines[line], shown[line]) != 0) {
            ESP_RETURN_ON_ERROR(lcd_print(TEXT_FIRST_ROW + line, lines[line]), TAG, "text line");
            strcpy(shown[line], lines[line]);
        }
    }
    return ESP_OK;
}

static enum clock_face nth_rotation_face(int64_t period)
{
    const uint8_t faces = display.rotate_faces;
    int remaining = period % __builtin_popcount(faces);
    for (int face = 0; face < FACE_COUNT; face++) {
        if ((faces >> face & 1) && remaining-- == 0) {
            return face;
        }
    }
    return FACE_BIG;
}

static enum clock_face choose_face(void)
{
    switch (display.mode) {
    case DISPLAY_MODE_BIG_DATE:
        return FACE_BIG_DATE;
    case DISPLAY_MODE_TEXT:
        return FACE_TEXT;
    case DISPLAY_MODE_ROTATE:
        return nth_rotation_face(esp_timer_get_time() / US_PER_SECOND / display.rotate_s);
    default:
        return FACE_BIG;
    }
}

static void clock_task(void *arg)
{
    enum screen_state state = SCREEN_BLANK;
    enum clock_face shown_face = FACE_BIG;
    char shown[DIGIT_COUNT] = {0};
    char shown_text[TEXT_LINE_COUNT][LCD_COLUMNS + 1] = {{0}};
    char shown_date[DATE_LINE_COUNT][DATE_LINE_SIZE] = {{0}};
    bool colons_shown = false;
    bool display_ready = false;
    int64_t last_recover_us = 0;

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_add(NULL));
    while (true) {
        esp_task_wdt_reset();
        if (!display_ready) {
            if (esp_timer_get_time() - last_recover_us >= RECOVER_INTERVAL_MS * 1000LL) {
                last_recover_us = esp_timer_get_time();
                display_ready = lcd_recover() == ESP_OK && define_glyphs() == ESP_OK;
                if (display_ready) {
                    startup_recovered("lcd");
                    state = SCREEN_BLANK;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
            continue;
        }
        struct tm utc;
        esp_err_t err = rtc_read(&utc);
        esp_err_t drawn = ESP_OK;
        if (err == ESP_ERR_INVALID_STATE && state != SCREEN_WAITING) {
            drawn = lcd_clear();
            if (drawn == ESP_OK) {
                drawn = lcd_print(1, "  Waiting for time");
            }
            if (drawn == ESP_OK) {
                state = SCREEN_WAITING;
            }
        } else if (err == ESP_OK) {
            const enum clock_face face = choose_face();
            const bool force = state != SCREEN_CLOCK || face != shown_face;
            if (force) {
                drawn = lcd_clear();
            }
            const time_t seconds = timegm(&utc);
            struct tm local;
            localtime_r(&seconds, &local);
            if (drawn == ESP_OK && face == FACE_TEXT) {
                drawn = show_text(&local, shown_text, force);
            } else if (drawn == ESP_OK) {
                drawn = show_clock(&local, shown, &colons_shown, force, face == FACE_BIG);
                if (drawn == ESP_OK && face == FACE_BIG_DATE) {
                    drawn = show_date(&local, shown_date, force);
                }
            }
            if (drawn == ESP_OK) {
                state = SCREEN_CLOCK;
                shown_face = face;
            }
        }
        if (drawn != ESP_OK) {
            ESP_LOGW(TAG, "display write failed: %s", esp_err_to_name(drawn));
            display_ready = false;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

void bigclock_get_display(display_settings_t *out)
{
    *out = display;
}

esp_err_t bigclock_set_display(const display_settings_t *requested)
{
    ESP_RETURN_ON_FALSE(requested->rotate_s >= DISPLAY_ROTATE_MIN_S && requested->rotate_s <= DISPLAY_ROTATE_MAX_S,
                        ESP_ERR_INVALID_ARG, TAG, "rotation period");
    ESP_RETURN_ON_FALSE(requested->rotate_faces != 0 && (requested->rotate_faces & ~DISPLAY_FACES_ALL) == 0,
                        ESP_ERR_INVALID_ARG, TAG, "rotation screens");
    ESP_RETURN_ON_ERROR(settings_save_display(requested), TAG, "store display");
    display = *requested;
    return ESP_OK;
}

esp_err_t bigclock_start(void)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(settings_load_display(&display));
    ESP_RETURN_ON_FALSE(xTaskCreate(clock_task, "bigclock", 4096, NULL, 4, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

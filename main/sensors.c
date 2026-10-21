#include "sensors.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ONEWIRE_GPIO GPIO_NUM_4
// Interrupts are masked during each time slot; core 1 keeps that away from the PPS interrupt on core 0.
#define SENSOR_TASK_CORE 1
#define SENSOR_TASK_STACK 4096
#define SENSOR_TASK_PRIORITY 4

#define READ_INTERVAL_MS 10000
#define RESCAN_INTERVAL_US (60 * 1000000LL)
#define READING_MAX_AGE_US (3 * READ_INTERVAL_MS * 1000LL)
#define CONVERSION_TIME_MS 800

#define RESET_LOW_US 480
#define RESET_SAMPLE_DELAY_US 70
#define RESET_RECOVERY_US 410
#define WRITE_1_LOW_US 6
#define WRITE_1_HIGH_US 64
#define WRITE_0_LOW_US 60
#define WRITE_0_HIGH_US 10
#define READ_LOW_US 6
#define READ_SAMPLE_DELAY_US 9
#define READ_RECOVERY_US 55

#define CMD_SEARCH_ROM 0xF0
#define CMD_SKIP_ROM 0xCC
#define CMD_MATCH_ROM 0x55
#define CMD_CONVERT 0x44
#define CMD_READ_SCRATCHPAD 0xBE
#define FAMILY_DS18B20 0x28
#define SCRATCHPAD_SIZE 9
#define ROM_BITS 64
#define CRC8_POLYNOMIAL_REFLECTED 0x8C
#define POWER_ON_TEMPERATURE_RAW 0x0550
#define POWER_ON_COUNT_REMAIN 0x0C
#define DEGREES_PER_LSB 0.0625f
#define DEFAULT_NAME_SERIAL_BYTES 6

struct sensor {
    uint8_t rom[SETTINGS_ROM_LENGTH];
    char name[SETTINGS_SENSOR_NAME_MAX_LENGTH];
    bool custom_name;
    bool munin;
    bool present;
    bool has_reading;
    float temperature_c;
    int64_t reading_us;
};

static const char *TAG = "sensors";

static struct sensor table[SENSORS_MAX];
static size_t table_size;
static sensor_name_t stored_names[SENSORS_MAX];
static uint8_t munin_roms[SENSORS_MAX][SETTINGS_ROM_LENGTH];
static portMUX_TYPE table_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE bus_lock = portMUX_INITIALIZER_UNLOCKED;

static void bus_low(void)
{
    gpio_set_level(ONEWIRE_GPIO, 0);
}

static void bus_release(void)
{
    gpio_set_level(ONEWIRE_GPIO, 1);
}

static bool onewire_reset(void)
{
    bus_low();
    esp_rom_delay_us(RESET_LOW_US);
    portENTER_CRITICAL(&bus_lock);
    bus_release();
    esp_rom_delay_us(RESET_SAMPLE_DELAY_US);
    const bool presence = gpio_get_level(ONEWIRE_GPIO) == 0;
    portEXIT_CRITICAL(&bus_lock);
    esp_rom_delay_us(RESET_RECOVERY_US);
    return presence;
}

static void write_bit(bool bit)
{
    portENTER_CRITICAL(&bus_lock);
    bus_low();
    esp_rom_delay_us(bit ? WRITE_1_LOW_US : WRITE_0_LOW_US);
    bus_release();
    esp_rom_delay_us(bit ? WRITE_1_HIGH_US : WRITE_0_HIGH_US);
    portEXIT_CRITICAL(&bus_lock);
}

static bool read_bit(void)
{
    portENTER_CRITICAL(&bus_lock);
    bus_low();
    esp_rom_delay_us(READ_LOW_US);
    bus_release();
    esp_rom_delay_us(READ_SAMPLE_DELAY_US);
    const bool bit = gpio_get_level(ONEWIRE_GPIO) != 0;
    esp_rom_delay_us(READ_RECOVERY_US);
    portEXIT_CRITICAL(&bus_lock);
    return bit;
}

static void write_byte(uint8_t value)
{
    for (int i = 0; i < 8; i++) {
        write_bit((value >> i) & 1);
    }
}

static uint8_t read_byte(void)
{
    uint8_t value = 0;
    for (int i = 0; i < 8; i++) {
        value |= (uint8_t)(read_bit() << i);
    }
    return value;
}

static uint8_t crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 1) ? (uint8_t)((crc >> 1) ^ CRC8_POLYNOMIAL_REFLECTED) : (uint8_t)(crc >> 1);
        }
    }
    return crc;
}

struct search_state {
    uint8_t rom[SETTINGS_ROM_LENGTH];
    int last_discrepancy;
    bool done;
};

// Maxim AN187 ROM search: finds the next device, branching at the last unresolved discrepancy.
static bool search_next(struct search_state *state)
{
    if (state->done || !onewire_reset()) {
        state->done = true;
        return false;
    }
    write_byte(CMD_SEARCH_ROM);

    int bit_number = 1;
    int last_zero = 0;
    int byte_index = 0;
    uint8_t mask = 1;
    while (byte_index < SETTINGS_ROM_LENGTH) {
        const bool id_bit = read_bit();
        const bool complement = read_bit();
        if (id_bit && complement) {
            state->done = true;
            return false;
        }
        bool direction;
        if (id_bit != complement) {
            direction = id_bit;
        } else {
            direction = bit_number < state->last_discrepancy ? (state->rom[byte_index] & mask) != 0
                                                             : bit_number == state->last_discrepancy;
            if (!direction) {
                last_zero = bit_number;
            }
        }
        if (direction) {
            state->rom[byte_index] |= mask;
        } else {
            state->rom[byte_index] &= (uint8_t)~mask;
        }
        write_bit(direction);
        bit_number++;
        mask <<= 1;
        if (mask == 0) {
            byte_index++;
            mask = 1;
        }
    }
    state->last_discrepancy = last_zero;
    state->done = last_zero == 0;
    return bit_number > ROM_BITS && crc8(state->rom, SETTINGS_ROM_LENGTH - 1) == state->rom[SETTINGS_ROM_LENGTH - 1];
}

static void rom_to_id(const uint8_t rom[SETTINGS_ROM_LENGTH], char id[SENSOR_ID_LENGTH])
{
    for (int i = 0; i < SETTINGS_ROM_LENGTH; i++) {
        snprintf(id + 2 * i, 3, "%02x", rom[i]);
    }
}

static void default_name(const uint8_t rom[SETTINGS_ROM_LENGTH], char name[SETTINGS_SENSOR_NAME_MAX_LENGTH])
{
    int length = snprintf(name, SETTINGS_SENSOR_NAME_MAX_LENGTH, "ds18b20_");
    for (int i = 1; i <= DEFAULT_NAME_SERIAL_BYTES; i++) {
        length += snprintf(name + length, SETTINGS_SENSOR_NAME_MAX_LENGTH - length, "%02x", rom[i]);
    }
}

static bool rom_unused(const uint8_t rom[SETTINGS_ROM_LENGTH])
{
    for (int i = 0; i < SETTINGS_ROM_LENGTH; i++) {
        if (rom[i] != 0) {
            return false;
        }
    }
    return true;
}

static bool published_in_munin(const uint8_t rom[SETTINGS_ROM_LENGTH])
{
    for (size_t i = 0; i < SENSORS_MAX; i++) {
        if (memcmp(munin_roms[i], rom, SETTINGS_ROM_LENGTH) == 0) {
            return true;
        }
    }
    return false;
}

bool sensors_name_valid(const char *name)
{
    const size_t length = strlen(name);
    if (length == 0 || length >= SETTINGS_SENSOR_NAME_MAX_LENGTH || isdigit((unsigned char)name[0])) {
        return false;
    }
    for (size_t i = 0; i < length; i++) {
        if (!isalnum((unsigned char)name[i]) && name[i] != '_') {
            return false;
        }
    }
    return true;
}

static struct sensor *find_by_rom(const uint8_t rom[SETTINGS_ROM_LENGTH])
{
    for (size_t i = 0; i < table_size; i++) {
        if (memcmp(table[i].rom, rom, SETTINGS_ROM_LENGTH) == 0) {
            return &table[i];
        }
    }
    return NULL;
}

static const char *stored_name_for(const uint8_t rom[SETTINGS_ROM_LENGTH])
{
    for (size_t i = 0; i < SENSORS_MAX; i++) {
        if (stored_names[i].name[0] && memcmp(stored_names[i].rom, rom, SETTINGS_ROM_LENGTH) == 0) {
            return stored_names[i].name;
        }
    }
    return NULL;
}

static void register_found(const uint8_t rom[SETTINGS_ROM_LENGTH])
{
    portENTER_CRITICAL(&table_lock);
    struct sensor *known = find_by_rom(rom);
    if (known) {
        known->present = true;
    } else if (table_size < SENSORS_MAX) {
        struct sensor *entry = &table[table_size++];
        *entry = (struct sensor){.present = true};
        memcpy(entry->rom, rom, SETTINGS_ROM_LENGTH);
        const char *stored = stored_name_for(rom);
        if (stored) {
            snprintf(entry->name, sizeof(entry->name), "%s", stored);
            entry->custom_name = true;
        } else {
            default_name(rom, entry->name);
        }
        entry->munin = published_in_munin(rom);
    }
    portEXIT_CRITICAL(&table_lock);
    if (!known) {
        char id[SENSOR_ID_LENGTH];
        rom_to_id(rom, id);
        ESP_LOGI(TAG, "found %s", id);
    }
}

static void rescan(void)
{
    portENTER_CRITICAL(&table_lock);
    for (size_t i = 0; i < table_size; i++) {
        table[i].present = false;
    }
    portEXIT_CRITICAL(&table_lock);

    struct search_state state = {0};
    while (search_next(&state)) {
        if (state.rom[0] == FAMILY_DS18B20) {
            register_found(state.rom);
        }
    }
}

static bool read_scratchpad(const uint8_t rom[SETTINGS_ROM_LENGTH], uint8_t scratchpad[SCRATCHPAD_SIZE])
{
    if (!onewire_reset()) {
        return false;
    }
    write_byte(CMD_MATCH_ROM);
    for (int i = 0; i < SETTINGS_ROM_LENGTH; i++) {
        write_byte(rom[i]);
    }
    write_byte(CMD_READ_SCRATCHPAD);
    for (int i = 0; i < SCRATCHPAD_SIZE; i++) {
        scratchpad[i] = read_byte();
    }
    return crc8(scratchpad, SCRATCHPAD_SIZE - 1) == scratchpad[SCRATCHPAD_SIZE - 1];
}

static void store_reading(size_t index, bool valid, float temperature_c)
{
    portENTER_CRITICAL(&table_lock);
    table[index].has_reading = valid;
    if (valid) {
        table[index].temperature_c = temperature_c;
        table[index].reading_us = esp_timer_get_time();
    }
    portEXIT_CRITICAL(&table_lock);
}

static void read_all(void)
{
    if (!onewire_reset()) {
        for (size_t i = 0; i < table_size; i++) {
            store_reading(i, false, 0);
        }
        return;
    }
    write_byte(CMD_SKIP_ROM);
    write_byte(CMD_CONVERT);
    vTaskDelay(pdMS_TO_TICKS(CONVERSION_TIME_MS));

    for (size_t i = 0; i < table_size; i++) {
        uint8_t scratchpad[SCRATCHPAD_SIZE];
        bool valid = table[i].present && read_scratchpad(table[i].rom, scratchpad);
        const int16_t raw = valid ? (int16_t)((scratchpad[1] << 8) | scratchpad[0]) : 0;
        // The scratchpad keeps this value until the first conversion completes.
        if (valid && raw == POWER_ON_TEMPERATURE_RAW && scratchpad[6] == POWER_ON_COUNT_REMAIN) {
            valid = false;
        }
        store_reading(i, valid, raw * DEGREES_PER_LSB);
    }
}

static void sensor_task(void *arg)
{
    int64_t last_scan_us = 0;
    bool scanned = false;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_task_wdt_add(NULL));
    while (true) {
        esp_task_wdt_reset();
        const int64_t now_us = esp_timer_get_time();
        if (!scanned || now_us - last_scan_us >= RESCAN_INTERVAL_US) {
            rescan();
            last_scan_us = now_us;
            scanned = true;
        }
        read_all();
        vTaskDelay(pdMS_TO_TICKS(READ_INTERVAL_MS));
    }
}

size_t sensors_snapshot(sensor_info_t out[SENSORS_MAX])
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&table_lock);
    const size_t count = table_size;
    for (size_t i = 0; i < count; i++) {
        rom_to_id(table[i].rom, out[i].id);
        snprintf(out[i].name, sizeof(out[i].name), "%s", table[i].name);
        out[i].munin = table[i].munin;
        out[i].present = table[i].present;
        out[i].has_reading = table[i].present && table[i].has_reading
                             && now_us - table[i].reading_us < READING_MAX_AGE_US;
        out[i].temperature_c = table[i].temperature_c;
    }
    portEXIT_CRITICAL(&table_lock);
    return count;
}

static struct sensor *find_by_id(const char *id)
{
    for (size_t i = 0; i < table_size; i++) {
        char candidate[SENSOR_ID_LENGTH];
        rom_to_id(table[i].rom, candidate);
        if (strcmp(candidate, id) == 0) {
            return &table[i];
        }
    }
    return NULL;
}

esp_err_t sensors_rename(const char *id, const char *name)
{
    const bool use_default = name[0] == '\0';
    if (!use_default && !sensors_name_valid(name)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = ESP_ERR_NOT_FOUND;
    sensor_name_t to_store[SENSORS_MAX] = {0};
    portENTER_CRITICAL(&table_lock);
    struct sensor *target = find_by_id(id);
    if (target) {
        result = ESP_OK;
        char new_name[SETTINGS_SENSOR_NAME_MAX_LENGTH];
        if (use_default) {
            default_name(target->rom, new_name);
        } else {
            snprintf(new_name, sizeof(new_name), "%s", name);
        }
        for (size_t i = 0; i < table_size; i++) {
            if (&table[i] != target && strcmp(table[i].name, new_name) == 0) {
                result = ESP_ERR_INVALID_STATE;
            }
        }
        if (result == ESP_OK) {
            snprintf(target->name, sizeof(target->name), "%s", new_name);
            target->custom_name = !use_default;
            size_t used = 0;
            for (size_t i = 0; i < table_size; i++) {
                if (table[i].custom_name) {
                    memcpy(to_store[used].rom, table[i].rom, SETTINGS_ROM_LENGTH);
                    snprintf(to_store[used].name, sizeof(to_store[used].name), "%s", table[i].name);
                    used++;
                }
            }
        }
    }
    portEXIT_CRITICAL(&table_lock);

    if (result == ESP_OK) {
        memcpy(stored_names, to_store, sizeof(stored_names));
        ESP_RETURN_ON_ERROR(settings_save_sensor_names(to_store), TAG, "store names");
    }
    return result;
}

esp_err_t sensors_set_munin(const char *id, bool enabled)
{
    esp_err_t result = ESP_ERR_NOT_FOUND;
    uint8_t to_store[SENSORS_MAX][SETTINGS_ROM_LENGTH];
    portENTER_CRITICAL(&table_lock);
    struct sensor *target = find_by_id(id);
    if (target) {
        result = ESP_OK;
        for (size_t i = 0; i < SENSORS_MAX; i++) {
            if (memcmp(munin_roms[i], target->rom, SETTINGS_ROM_LENGTH) == 0) {
                memset(munin_roms[i], 0, SETTINGS_ROM_LENGTH);
            }
        }
        if (enabled) {
            result = ESP_ERR_NO_MEM;
            for (size_t i = 0; i < SENSORS_MAX && result != ESP_OK; i++) {
                if (rom_unused(munin_roms[i])) {
                    memcpy(munin_roms[i], target->rom, SETTINGS_ROM_LENGTH);
                    result = ESP_OK;
                }
            }
        }
        if (result == ESP_OK) {
            target->munin = enabled;
            memcpy(to_store, munin_roms, sizeof(to_store));
        }
    }
    portEXIT_CRITICAL(&table_lock);

    if (result == ESP_OK) {
        ESP_RETURN_ON_ERROR(settings_save_munin_sensors(to_store), TAG, "store munin sensors");
    }
    return result;
}

esp_err_t sensors_start(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << ONEWIRE_GPIO,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio");
    bus_release();
    ESP_RETURN_ON_ERROR(settings_load_sensor_names(stored_names), TAG, "load names");
    ESP_RETURN_ON_ERROR(settings_load_munin_sensors(munin_roms), TAG, "load munin sensors");
    ESP_RETURN_ON_FALSE(xTaskCreatePinnedToCore(sensor_task, "sensors", SENSOR_TASK_STACK, NULL,
                                                SENSOR_TASK_PRIORITY, NULL, SENSOR_TASK_CORE) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

#include "startup.h"

#include <string.h>

#include "freertos/FreeRTOS.h"

static const char *failed[STARTUP_MAX_FAILURES];
static size_t failed_count;
static portMUX_TYPE failed_lock = portMUX_INITIALIZER_UNLOCKED;

void startup_failed(const char *component)
{
    portENTER_CRITICAL(&failed_lock);
    if (failed_count < STARTUP_MAX_FAILURES) {
        failed[failed_count++] = component;
    }
    portEXIT_CRITICAL(&failed_lock);
}

void startup_recovered(const char *component)
{
    portENTER_CRITICAL(&failed_lock);
    for (size_t i = 0; i < failed_count; i++) {
        if (strcmp(failed[i], component) == 0) {
            failed[i] = failed[--failed_count];
            break;
        }
    }
    portEXIT_CRITICAL(&failed_lock);
}

size_t startup_get_failures(const char *components[STARTUP_MAX_FAILURES])
{
    portENTER_CRITICAL(&failed_lock);
    const size_t count = failed_count;
    memcpy(components, failed, count * sizeof(failed[0]));
    portEXIT_CRITICAL(&failed_lock);
    return count;
}

#pragma once

#include <stddef.h>

#define STARTUP_MAX_FAILURES 16

// Components that did not start. The device keeps running without them, and the web page lists them.
// The name must stay valid for the lifetime of the program.
void startup_failed(const char *component);
void startup_recovered(const char *component);

size_t startup_get_failures(const char *components[STARTUP_MAX_FAILURES]);

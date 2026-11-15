#pragma once

#include "esp_err.h"

// Serves the gpsd JSON protocol on TCP 2947: after ?WATCH a client receives a TPV and a SKY report every second,
// half a second apart.
// Only ?WATCH, ?VERSION and ?DEVICES are understood.
esp_err_t gpsd_start(void);

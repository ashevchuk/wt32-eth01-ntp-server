#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "settings.h"

typedef struct {
    // Counted by the PHY and the EMAC since boot.
    uint32_t phy_symbol_errors;
    uint32_t emac_missed_frames;
    uint32_t emac_fifo_overflows;
    // Negotiated link; zero speed while the link is down.
    uint32_t link_speed_mbps;
    bool full_duplex;
} eth_diagnostics_t;

esp_err_t eth_start(const network_settings_t *network);
void eth_get_diagnostics(eth_diagnostics_t *diagnostics);

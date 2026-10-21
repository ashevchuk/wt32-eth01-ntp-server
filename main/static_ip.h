#pragma once

#include "esp_err.h"
#include "esp_netif.h"

// Stops DHCP on the interface and gives it a fixed address. An empty gateway or DNS leaves it unset.
esp_err_t static_ip_apply(esp_netif_t *netif, const char *ip, const char *mask, const char *gateway, const char *dns);

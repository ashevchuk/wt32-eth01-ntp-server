#include "eth.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "esp_eth_com.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/emac_dma_struct.h"

#include "static_ip.h"

#define ETH_OSC_ENABLE_GPIO GPIO_NUM_16
#define ETH_PHY_ADDR 1
#define ETH_OSC_STARTUP_MS 10

#define PHY_REG_SYMBOL_ERROR_COUNTER 26
#define DIAGNOSTICS_INTERVAL_MS 1000
#define DIAGNOSTICS_TASK_STACK 3072

static const char *TAG = "eth";

static esp_eth_handle_t eth_handle;
static eth_diagnostics_t diagnostics;
static portMUX_TYPE diagnostics_lock = portMUX_INITIALIZER_UNLOCKED;

static void on_eth_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "link up");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "link down");
        break;
    default:
        break;
    }
}

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const ip_event_got_ip_t *event = data;
    ESP_LOGI(TAG, "ip " IPSTR " mask " IPSTR " gw " IPSTR,
             IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.netmask), IP2STR(&event->ip_info.gw));
}

static esp_err_t enable_phy_oscillator(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << ETH_OSC_ENABLE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "oscillator gpio config");
    ESP_RETURN_ON_ERROR(gpio_set_level(ETH_OSC_ENABLE_GPIO, 1), TAG, "oscillator enable");
    vTaskDelay(pdMS_TO_TICKS(ETH_OSC_STARTUP_MS));
    return ESP_OK;
}

static bool read_phy_symbol_errors(uint32_t *value)
{
    uint32_t raw = 0;
    esp_eth_phy_reg_rw_data_t access = {.reg_addr = PHY_REG_SYMBOL_ERROR_COUNTER, .reg_value_p = &raw};
    if (esp_eth_ioctl(eth_handle, ETH_CMD_READ_PHY_REG, &access) != ESP_OK) {
        return false;
    }
    *value = raw;
    return true;
}

// Both EMAC counters clear on read. The PHY counter is only followed while it grows.
static void diagnostics_task(void *arg)
{
    uint32_t previous_symbol_errors = 0;
    while (true) {
        const uint32_t missed = EMAC_DMA.dmamissedfr.missed_fc;
        const uint32_t overflows = EMAC_DMA.dmamissedfr.overflow_fc;
        uint32_t symbol_errors;
        const bool have_symbol_errors = read_phy_symbol_errors(&symbol_errors);

        portENTER_CRITICAL(&diagnostics_lock);
        diagnostics.emac_missed_frames += missed;
        diagnostics.emac_fifo_overflows += overflows;
        if (have_symbol_errors) {
            diagnostics.phy_symbol_errors += symbol_errors >= previous_symbol_errors ? symbol_errors - previous_symbol_errors
                                                                                     : symbol_errors;
            previous_symbol_errors = symbol_errors;
        }
        portEXIT_CRITICAL(&diagnostics_lock);
        vTaskDelay(pdMS_TO_TICKS(DIAGNOSTICS_INTERVAL_MS));
    }
}

void eth_get_diagnostics(eth_diagnostics_t *out)
{
    portENTER_CRITICAL(&diagnostics_lock);
    *out = diagnostics;
    portEXIT_CRITICAL(&diagnostics_lock);

    eth_speed_t speed;
    eth_duplex_t duplex;
    const bool known = eth_handle && esp_eth_ioctl(eth_handle, ETH_CMD_G_SPEED, &speed) == ESP_OK
                       && esp_eth_ioctl(eth_handle, ETH_CMD_G_DUPLEX_MODE, &duplex) == ESP_OK;
    out->link_speed_mbps = known ? (speed == ETH_SPEED_100M ? 100 : 10) : 0;
    out->full_duplex = known && duplex == ETH_DUPLEX_FULL;
}

esp_err_t eth_start(const network_settings_t *network)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");
    ESP_RETURN_ON_ERROR(enable_phy_oscillator(), TAG, "phy oscillator");

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&netif_cfg);
    ESP_RETURN_ON_FALSE(netif, ESP_ERR_NO_MEM, TAG, "netif alloc");
    ESP_RETURN_ON_ERROR(esp_netif_set_hostname(netif, network->hostname), TAG, "hostname");
    if (network->eth_static) {
        ESP_RETURN_ON_ERROR(static_ip_apply(netif, network->eth_ip, network->eth_mask, network->eth_gateway, network->eth_dns),
                            TAG, "static address");
    }

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
    ESP_RETURN_ON_FALSE(mac, ESP_FAIL, TAG, "mac create");

    // The default reset_gpio_num is 5, which is the GNSS UART RX pin here.
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = ETH_PHY_ADDR;
    phy_cfg.reset_gpio_num = -1;
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_cfg);
    ESP_RETURN_ON_FALSE(phy, ESP_FAIL, TAG, "phy create");

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth = NULL;
    ESP_RETURN_ON_ERROR(esp_eth_driver_install(&eth_cfg, &eth), TAG, "driver install");
    eth_handle = eth;
    ESP_RETURN_ON_ERROR(esp_netif_attach(netif, esp_eth_new_netif_glue(eth)), TAG, "netif attach");

    ESP_RETURN_ON_ERROR(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, on_eth_event, NULL), TAG, "eth handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_got_ip, NULL), TAG, "ip handler");

    ESP_RETURN_ON_ERROR(esp_eth_start(eth), TAG, "start");
    ESP_RETURN_ON_FALSE(xTaskCreate(diagnostics_task, "eth_diag", DIAGNOSTICS_TASK_STACK, NULL, 3, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "diagnostics task");
    return ESP_OK;
}

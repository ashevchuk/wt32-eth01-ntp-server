#include "static_ip.h"

#include "esp_check.h"

static const char *TAG = "static_ip";

esp_err_t static_ip_apply(esp_netif_t *netif, const char *ip, const char *mask, const char *gateway, const char *dns)
{
    const esp_err_t stop_err = esp_netif_dhcpc_stop(netif);
    if (stop_err != ESP_OK && stop_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        return stop_err;
    }
    esp_netif_ip_info_t info = {0};
    ESP_RETURN_ON_ERROR(esp_netif_str_to_ip4(ip, &info.ip), TAG, "ip");
    ESP_RETURN_ON_ERROR(esp_netif_str_to_ip4(mask, &info.netmask), TAG, "netmask");
    if (gateway[0]) {
        ESP_RETURN_ON_ERROR(esp_netif_str_to_ip4(gateway, &info.gw), TAG, "gateway");
    }
    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(netif, &info), TAG, "set ip info");

    if (dns[0]) {
        esp_netif_dns_info_t dns_info = {0};
        dns_info.ip.type = ESP_IPADDR_TYPE_V4;
        ESP_RETURN_ON_ERROR(esp_netif_str_to_ip4(dns, &dns_info.ip.u_addr.ip4), TAG, "dns");
        ESP_RETURN_ON_ERROR(esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info), TAG, "set dns");
    }
    return ESP_OK;
}

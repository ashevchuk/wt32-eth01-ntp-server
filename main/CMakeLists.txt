idf_component_register(SRCS "main.c" "bigclock.c" "eth.c" "gnss.c" "ubx.c" "gnss_config.c" "leap.c" "i2c_bus.c" "lcd.c" "rtc.c" "ntp_clock.c" "ntp_server.c" "web.c" "settings.c" "wifi.c" "ota.c" "sensors.c" "munin.c" "upstream.c" "reset_button.c" "static_ip.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_eth esp_netif esp_event esp_driver_gpio esp_driver_uart esp_timer esp_driver_i2c esp_rom lwip esp_http_server esp_system esp_app_format nvs_flash esp_wifi app_update
                       EMBED_TXTFILES "web/index.html")

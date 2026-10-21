# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Network NTP server firmware for the WT32-ETH01 board (ESP32, wired Ethernet), built with ESP-IDF. The repository holds a minimal ESP-IDF project skeleton (`main/main.c` only logs a start message); the sections below record the hardware and toolchain facts the code must be written against.

## Hardware

| Peripheral | Interface | Pins |
|---|---|---|
| DS3231M RTC | I2C | SDA IO33, SCL IO32 |
| HD44780 20x4 LCD via PCF8574 backpack | I2C (same bus as the RTC) | SDA IO33, SCL IO32 |
| u-blox M8N GNSS | UART, 115200 baud, NMEA | TX IO17, RX IO5 |
| GNSS PPS | GPIO input | IO35 |
| Reset button (to GND, internal pull-up): hold 5 s = Ethernet back to DHCP | GPIO input | IO14 |
| DS18B20 sensors (external power, 4.7 kΩ pull-up) | 1-Wire bus | IO4 |

- The RTC and the LCD share one I2C bus; their addresses differ (DS3231 `0x68`, PCF8574 `0x27` or `0x3F`).
- IO35 is input-only and has no internal pull-up.
- Ethernet on this board uses GPIO0 as the RMII clock input and GPIO16 as the oscillator enable; do not reuse them.
- LCD backpacks are usually powered from 5 V, and their I2C pull-ups may pull the bus above the ESP32's 3.3 V tolerance.

## Architecture

- `gnss.c` parses RMC sentences (checksum-verified, fix status `A` only) and timestamps PPS edges in an ISR. The pairing is time-critical: RMC describes the second that began at the previous pulse, so `pps_task` takes the pending RMC at the next pulse, adds one second and calls the `gnss_start` callback, discarding RMC samples that arrived more than 500 ms after the previous pulse.
- `main.c` uses that callback to write the RTC at first fix and then hourly.
- `ntp_clock.c` keeps three anchors (unix time at an `esp_timer` instant): GNSS, network and RTC, and extrapolates from the best one. Priority: GNSS while it is less than 10 minutes old (stratum 1, refid `GPS`), then the network while it is less than 45 minutes old (stratum = upstream + 1, refid = upstream IPv4), otherwise the newest anchor of any kind (stratum 10, refid `LOCL`); with no anchor the server reports stratum 16, LI=3 and zero timestamps. A task re-anchors to the RTC at the instant its seconds register changes, once a minute, while neither GNSS nor the network is usable.
- `upstream.c` is a minimal SNTP client (no lwIP SNTP): it resolves the configured server (default `pool.ntp.org`, NVS key `upstream`, empty disables it), takes four samples 0.5 s apart every 15 minutes and keeps the one with the lowest round trip, applying half the round trip as correction. It also records how far GNSS time was from the upstream and, when GNSS is not the source, writes the RTC on a second boundary at most once an hour.
- `leap.c` polls `UBX-AID-HUI` once a minute and keeps the GPS-UTC offset and any scheduled leap second as an `esp_timer` instant. The usual `UBX-NAV-TIMELS` is not used because this receiver's firmware (SPG 2.01, protocol 15) never answers it; `AID-HUI` reports the satellite-broadcast offset, the scheduled offset and the week and day of the change. The week field is expanded to a full week number near the present, so it is meaningful only while the scheduled offset differs from the current one; `ntp_server.c` asks it for the LI bits and `/api/status` (`gnss.leap`) shows the state. The receiver reports an inserted second as `23:59:60`, and `gnss.c` does not add a second to it, so the anchor after it is midnight. During the inserted second itself the clock is not stepped back, so replies in that second are one second ahead.
- `ubx.c` frames UBX messages out of the same UART byte stream as NMEA (`ubx_feed` is called first for every byte) and offers `ubx_poll`/`ubx_set`. `gnss_config.c` uses it a few seconds after boot to make the M8N use the stationary dynamic model (`UBX-CFG-NAV5`) and, once the user has set them in the web page, the antenna cable delay and an additional PPS delay (`UBX-CFG-TP5`). It reads the current values first, writes only on a difference and then saves to the receiver's memory (`UBX-CFG-CFG`), because every save costs flash cycles. If the receiver does not answer, the UART's UBX input is probably disabled.
- `ntp_server.c` answers NTP client requests (mode 3, versions 1-4) on UDP 123 from a single task; the receive timestamp is taken right after `recvfrom`, the transmit timestamp right before `sendto`. The leap indicator is 1 or 2 only during the UTC day that ends with a leap second scheduled in the receiver (`leap.c`); it is 0 otherwise, also for network-sourced time, whose upstream indicator is not forwarded.
- `gnss.c` also keeps a status snapshot (GGA/GSA/GSV/RMC: fix, satellites, DOP, position) returned by `gnss_get_status`.
- `gnss.c` also keeps the satellite list of the last complete GSV cycle (`gnss_get_satellites`, served at `/api/gnss/satellites` in chunks because it does not fit the status buffer) and statistics of the PPS interval over the last minute (`gnss_get_pps`, in `gnss.pps`: offset in ppm of the ESP32 clock against the pulses, standard deviation and range; intervals more than 10 ms off are treated as missed pulses and ignored). A satellite counts as used when its number appears in a GSA sentence since the last RMC. This receiver speaks NMEA 4.0: GSA has no system ID and satellite numbers share one space (GPS 1-32, SBAS 33-64), and GSV leaves elevation and azimuth empty while the receiver has no orbit data for that satellite, so the sky plot shows only satellites with a direction while the bar chart shows all of them. All 16 HTTP handler slots (`HTTP_MAX_URI_HANDLERS`) are in use.
- All sockets come from one lwIP pool (`CONFIG_LWIP_MAX_SOCKETS=16` in `sdkconfig.defaults`): the HTTP server may hold 8 connections and evicts the least recently used one when full, because browsers keep several idle keep-alive connections per tab and would otherwise exhaust the pool, which also makes Munin and every other request fail with a connection reset.
- `web.c` serves `main/web/index.html` (embedded via `EMBED_TXTFILES`) and `GET /api/status`. The web interface has no authentication by owner's decision. JSON is written by hand with `snprintf` because IDF 6.0 no longer ships cJSON in core; adding it would be a new managed dependency.
- `settings.c` wraps NVS (namespace `ntp`) and applies the stored POSIX TZ string via `setenv`/`tzset`. Only the LCD clock and the web page use local time; NTP, RTC and the anchor always work in UTC, so use `timegm`/`gmtime_r` there, never `mktime`.
- Network settings (hostname, DHCP/static addressing for Ethernet and for Wi-Fi, Wi-Fi STA credentials) are one NVS blob in `settings.c`, which discards a stored blob of a different size, so changing `network_settings_t` resets them to the defaults; validated by `settings_network_error`, and applied only at boot by `eth_start` and `wifi_start`; `POST /api/reboot` restarts the device. `static_ip.c` applies a fixed address to either interface. Wi-Fi gets route priority 10 so Ethernet (50) stays the default route. Wi-Fi password is stored in plain NVS and never returned by the API.
- OTA: `ota.c` handles `POST /api/ota` (raw image as body), accepts only images whose app descriptor has this project's name, writes the next slot and reboots. Two 1.875 MB slots in `partitions.csv` on 4 MB flash; `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` makes the bootloader revert an image that resets before `ota_confirm_running_image()` runs at the end of `app_main`. Bump `PROJECT_VER` in the top-level `CMakeLists.txt` for each release so the running slot can be told apart.
- `sensors.c` bit-bangs 1-Wire on IO4 in a task pinned to core 1: interrupts are masked during every time slot, and keeping that on core 1 leaves the PPS interrupt on core 0 undisturbed. It rescans the bus every 60 s (ROM search, CRC-checked), reads all DS18B20s every 10 s with one broadcast convert, and keeps unplugged sensors in the table as absent. Sensor names are Munin field names (`[A-Za-z_][A-Za-z0-9_]*`), stored per ROM code in NVS. A sensor without a name (the default for a new one) is listed in the web page but left out of the Munin reply; setting the name empty removes it again.
- `reset_button.c` polls IO14; a 5 s hold stores `eth_static = false` and restarts, and does nothing when Ethernet already uses DHCP. Wi-Fi settings and hostname are left alone.
- `munin.c` is a munin-node on TCP 4949 with one plugin (name stored in NVS, default `environment`, changeable at runtime), one field per named sensor (`U` when it has no fresh reading). It has no access control.
- `POST /api/time` (form field `utc`) writes the RTC and calls `ntp_clock_anchor_local`, which makes the server report stratum 10; it is rejected with 409 while GNSS or network time is available, because those anchors outrank it.
- `eth.c` also samples receive-error counters once a second: the LAN8720 symbol error counter (PHY register 26, meaningful only on a 100 Mbit link) and the EMAC missed-frame and FIFO-overflow counters. They and the negotiated speed and duplex appear in `/api/status` and on the Ethernet card. Missed/overflow staying at zero while packets are lost means the loss happens on the wire, not in the ESP32's buffers. `/api/status` also reports the last reset reason.
- Watchdogs: `sdkconfig.defaults` makes the task watchdog panic (and therefore reboot) after 20 s; by default it only prints a warning. `ntp_server`, `bigclock`, `sensors` and `ntp_rtc` subscribe to it and feed it every loop pass, so the timeout must exceed the longest pass (the sensor task sleeps 10 s per pass). `ntp_server` uses a 1 s receive timeout so a quiet network does not look like a hang. Tasks that legitimately wait indefinitely (`pps`, `nmea`, `munin` accept, `upstream`) are not subscribed, and a stuck HTTP server task is not detected. `system.reset_reason` shows `task watchdog` after such a reset.
- `i2c_bus.c` owns the single I2C bus; `lcd.c` and `rtc.c` attach devices to it.
- `bigclock.c` polls the RTC every 100 ms and renders local time `HH:MM:SS` in 3x4-cell digits built from 8 custom characters, ported from lcdproc's BigClock (`clients/lcdproc/chrono.c`, `server/drivers/adv_bignum.c`); it redraws only changed digits, and the colons blink with second parity.
- DS3231 oscillator-stopped flag: `rtc_read` returns `ESP_ERR_INVALID_STATE` while it is set; `rtc_write` clears it.

## Toolchain

- ESP-IDF is installed at `/opt/esp-idf`; its tools are in `~/.espressif` (Python envs for IDF 5.5 and 6.0; the skeleton was built with 6.0). `idf.py` is not on `PATH` until the environment is exported: `. /opt/esp-idf/export.sh`.
- The board is attached as `/dev/ttyACM0` (group `uucp`, which the user belongs to).

## Commands

```
scripts/build.sh                      # idf.py build (runs set-target esp32 when sdkconfig is missing)
scripts/flash-serial.sh [port]        # flash over serial, default /dev/ttyACM0; needed after partition changes
scripts/flash-ota.sh <host> [image]   # upload build/ntp.bin to the running device and wait for the new slot
idf.py menuconfig                     # after sourcing /opt/esp-idf/export.sh
```

`flash-ota.sh` exits non-zero when the upload is rejected, the device does not return, or it returns on the same slot (the new image was rolled back).

`sdkconfig` is generated from `sdkconfig.defaults` (flash size, partition table, rollback); change the defaults and rerun `idf.py set-target esp32` rather than editing `sdkconfig` by hand. Moving to this partition layout required one flash over the serial port.

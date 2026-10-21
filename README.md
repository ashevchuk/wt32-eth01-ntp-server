# NTP server on WT32-ETH01

Stratum 1 NTP server for the WT32-ETH01 (ESP32, wired Ethernet), disciplined by a u-blox M8N GNSS receiver with a PPS output and backed by a DS3231M RTC. A 20x4 LCD shows the local time, DS18B20 sensors report temperature to Munin, and a web interface covers status, settings and firmware updates.

## Pinout

All GPIO numbers refer to the ESP32. Check them against the silkscreen of your board before soldering.

| GPIO | Connected to | Direction | Notes |
|------|--------------|-----------|-------|
| IO32 | I2C SCL: DS3231M, PCF8574 | bidirectional | shared bus, 100 kHz |
| IO33 | I2C SDA: DS3231M, PCF8574 | bidirectional | shared bus, 100 kHz |
| IO17 | GNSS module RX | ESP32 TX | UART1, 115200 baud, 8N1 |
| IO5  | GNSS module TX | ESP32 RX | UART1 |
| IO35 | GNSS module PPS | input | input-only pin, no internal pull-up; PPS drives it |
| IO4  | DS18B20 data (DQ) | bidirectional | 1-Wire, needs a 4.7 kΩ pull-up to 3.3 V |
| IO14 | Reset button to GND | input | internal pull-up; hold 5 s to restore DHCP on Ethernet |

I2C addresses: DS3231M `0x68`, PCF8574 LCD backpack `0x27` (the firmware also tries `0x3F`).

### Used by the board, do not connect anything

| GPIO | Function |
|------|----------|
| IO0  | Ethernet RMII reference clock (input from the on-board oscillator) |
| IO16 | Ethernet oscillator enable |
| IO18 | Ethernet MDIO |
| IO23 | Ethernet MDC |
| IO19, IO21, IO22, IO25, IO26, IO27 | Ethernet RMII data lines |
| IO1, IO3 | UART0: serial console and flashing |

### Avoid

| GPIO | Reason |
|------|--------|
| IO2, IO12, IO15 | Strapping pins, a wrong level at power-up changes the boot mode |
| IO36, IO39 | Input only |

## Wiring

### GNSS (u-blox M8N)

```
ESP32            M8N
IO17 (TX)  --->  RX
IO5  (RX)  <---  TX
IO35       <---  PPS
3V3        ----  VCC
GND        ----  GND
```

TX and RX are crossed. The firmware expects NMEA sentences at 115200 baud and tells the receiver that the antenna is active (`ITFM_ANTENNA_ACTIVE` in `main/gnss_config.c`), which the interference monitor needs to judge the signal level; change it for a passive antenna. Place the antenna with a clear view of the sky; with a poor view the fix flickers and the server falls back to the RTC.

### I2C bus (RTC and LCD)

```
ESP32            DS3231M          PCF8574 LCD backpack
IO33 (SDA) ----- SDA ------------ SDA
IO32 (SCL) ----- SCL ------------ SCL
3V3        ----- VCC
GND        ----- GND ----------- GND
                                  VCC (see the note below)
```

The RTC and the backpack share the same two wires. Both need pull-ups on SDA and SCL; the firmware also enables the internal ones.

**Voltage warning.** LCD backpacks are usually powered from 5 V, and their on-board pull-ups then pull SDA and SCL up to 5 V, above the 3.3 V that ESP32 pins tolerate. Either remove the backpack's pull-ups or use a level shifter, or run the backpack from 3.3 V if the display stays readable.

### DS18B20 temperature sensors

```
3V3 ----+------------+------------ VDD  (every sensor)
        |            |
       4.7k          |
        |            |
IO4 ----+------------+------------ DQ   (every sensor)

GND ------------------------------ GND  (every sensor)
```

Any number of sensors (up to 16) go on the same three wires. They must have their own VDD connection; parasitic power is not supported.

### Reset button

```
IO14 ----[ button ]---- GND
```

No resistor is needed. Holding it for 5 seconds sets Ethernet back to DHCP and restarts the device; Wi-Fi settings, the hostname and everything else are kept.

## Building and flashing

See `scripts/build.sh`, `scripts/flash-serial.sh` and `scripts/flash-ota.sh`. The first flash, and any flash after a change of the partition table, must go over the serial port.

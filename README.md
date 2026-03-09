# Air quality monitor firmware

Firmware for the air quality monitor project I built. See the [blog post for more details](https://blog.jean-francois.im/2021/05/08/building-a-simple-air-quality-monitor/).

The firmware allows accessing the various sensor readings through a `/status` endpoint (returns JSON), by pushing them as CSV values to a [user configurable server that can log those](https://github.com/jfim/air-quality-server), or by logging them as CSV values over the USB serial port.

## Hardware

- **MCU**: ESP8266 (Wemos D1 Mini)
- **CO2 sensor**: Senseair S8 (Modbus RTU over software UART, pins D0/D7)
- **Particle sensor**: Plantower PMSA003 (software UART, pins D5/D6)
- **Temperature/humidity**: Sensirion SHT31 (I2C, address 0x44)
- **VOC**: Sensirion SGP30 (I2C, address 0x58)

Each sensor can be individually enabled/disabled via build flags in `platformio.ini`.

## Build flags

| Flag | Description |
|------|-------------|
| `ENABLE_CO2_SENSOR` | Enable Senseair S8 CO2 sensor |
| `ENABLE_PARTICLE_SENSOR` | Enable Plantower PMSA003 particle sensor |
| `ENABLE_SHT31` | Enable SHT31 temperature/humidity sensor |
| `ENABLE_SGP30` | Enable SGP30 eCO2/tVOC sensor |
| `ENABLE_NETWORK` | Enable WiFi, OTA updates, and web interface |
| `ENABLE_NETWORK_LOGGING` | Enable sending readings to a remote server via TCP |
| `ENABLE_SERIAL_DEBUGGING` | Enable debug output on serial |
| `ENABLE_SERIAL_LOGGING` | Enable CSV sensor readings on serial |

## Building and flashing

Requires [PlatformIO](https://platformio.org/) (CLI or VSCode extension).

```bash
# Build
pio run

# Flash via USB (set upload_protocol = esptool in platformio.ini)
pio run -t upload

# Flash via OTA (set upload_protocol = espota and upload_port to the device IP)
pio run -t upload
```

For initial flashing, use USB. Once `ENABLE_NETWORK` is set and the device is on WiFi, subsequent updates can be done over the air by setting `upload_protocol = espota` and `upload_port` to the device's IP address in `platformio.ini`.

## Configuration

On first boot (or after a factory reset), the device starts a WiFi captive portal where you can configure:

- WiFi credentials
- Server hostname (default: `air-quality-server.local`)
- Server port (default: `1234`)
- Temperature offset (default: `-3.4`)

Configuration is persisted to LittleFS and can be changed at any time via the web interface.

## Web interface

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Root page with links to all endpoints |
| `/status` | GET | JSON object with current config and sensor readings |
| `/config` | GET | Configuration form |
| `/config` | POST | Update configuration (hostname, port, temperature offset) |
| `/calibrate` | GET | Trigger Senseair S8 background calibration |
| `/factoryreset` | GET | Wipe config and WiFi credentials, reboot into captive portal |

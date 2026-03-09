# CLAUDE.md

## Project overview

ESP8266 (Wemos D1 Mini) air quality monitor firmware. Single-file project (`src/main.cpp`) that reads from multiple sensors and reports readings via serial, TCP, and a web interface.

## Build system

- PlatformIO with Arduino framework targeting `d1_mini`
- Build: `pio run`
- Flash: `pio run -t upload`
- Config in `platformio.ini`, sensors toggled via `-D` build flags

## Architecture

- All code is in `src/main.cpp` — no separate header files
- Sensors are conditionally compiled with `#ifdef` blocks
- Two software UARTs: CO2 sensor (D0/D7) and particle sensor (D5/D6)
- Two I2C devices: SHT31 (0x44) and SGP30 (0x58)
- Configuration stored in LittleFS as `/config.txt` (three lines: hostname, port, temperature offset)
- Web server runs on port 80 with endpoints for status, config, calibration, and factory reset

## Key conventions

- Sensor readings are stored in global `uint16_t` variables, reset to -1 (0xFFFF) each loop iteration
- -1 / 0xFFFF means "no reading available"
- checksumOk values: 1 = valid, 0 = invalid/error, 0xFFFF = no reading
- Debug output uses `DEBUG_PRINT` / `DEBUG_PRINT_2` macros (no-ops unless `ENABLE_SERIAL_DEBUGGING` is set)
- Network logging sends CSV over raw TCP to a configurable host:port

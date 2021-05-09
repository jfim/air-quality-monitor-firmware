# Air quality monitor firmware

Firmware for the air quality monitor project I built. See the blog post for more details.

## Building

You'll need [PlatformIO](https://platformio.org/) installed. The project can be built using the usual PlatformIO CLI or the VSCode extension.

## Flashing

For USB flashing, make sure to edit [platformio.ini](platformio.ini) to use the `esptool` upload method. Once flashed, the firmware can be updated over the air if `ENABLE_NETWORK` is defined.

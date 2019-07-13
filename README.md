# esp32-homekit-camera

Firmware for esp32-camera module to act as Apple Homekit IP camera.

Based on [esp-homekit](https://github.com/maximkulkin/esp-homekit).

## Configuration

Before compiling, you need to alter several settings in menuconfig (`make
menuconfig`):
* Partition Table
    * Partition Table = **Custom partition table CSV**
    * Custom partition CSV file = **partitions.csv**
* Component config
    * ESP32-specific
        * Support for external, SPI-connected RAM = **check**
        * SPI RAM config
            * Initialize SPI RAM when booting the ESP32 = **check**
            * SPI RAM access method = **Make RAM allocatable using malloc() as well**
    * Camera configuration
        * OV2640 Support = **check**
* ESP32 HomeKit Camera
    * AP or STA = *your preference*
    * Camera Pins
        * Select Camera Pinout = *your variant of module*

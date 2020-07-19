# esp32-homekit-camera

Firmware for esp32-camera module to act as Apple Homekit IP camera.

Based on [esp-homekit](https://github.com/maximkulkin/esp-homekit).

## Prerequisites

### esp-idf

- Install esp-idf by following instructions on esp-idf project page (https://github.com/espressif/esp-idf#setting-up-esp-idf). 
- At the end you should have xtensa-esp32-elf toolchain in your path and IDF_PATH environment variable pointing to esp-idf directory.

### esp32-homekit-camera code

- Clone this repository `git clone https://github.com/maximkulkin/esp32-homekit-camera.git`
- Work out of this directory going forward `cd esp32-homekit-camera`
- Initialize and sync all submodules (recursively) `git submodule update --init --recursive`

## Configuration

Before compiling, you need to alter several settings in **menuconfig**. Make sure you save
this file, as it will be used in the `build` section to configure ESP32-CAM as well as your
network/Homekit settings.

- Open the menuconfiguration with the command `make menuconfig`
- Scroll through each section of the menu, and make these changes:


* Serial flasher config
    * Default serial port
    * Flash size = **4 MB**
* Partition Table
    * Partition Table = **Custom partition table CSV**
    * Custom partition CSV file = **partitions.csv**
* Component config
    * Driver configuration
        * RTCIO configuration
            * Support array `rtc_gpio_desc` for ESP32 = **check**
    * ESP32-specific
        * Support for external, SPI-connected RAM = **check**
        * SPI RAM config
            * Initialize SPI RAM during startup = **check**
            * SPI RAM access method = **Make RAM allocatable using malloc() as well**
    * Camera configuration
        * OV2640 Support = **check**
    * HomeKit
        * SPI flash address for storing HomeKit data = **0x3A0000**
* ESP32 HomeKit Camera
    * WiFi SSID
    * WiFi Password
    * Select Camera Pinout
        * Select Camera Pinout = *your variant of module*
    * LED Pin
        * PIN = *your variant of module*
            * ESP32-CAM by AI-Thinker == **33**
    * Image sensor vertical flip *(optional)*
        * Toggle if HomeKit app has video feed upside down
    * Image sensor horizontal mirror *(optional)*
        * Toggle if HomeKit app has video feed mirrored
    * Image sensor enhance settings *(optional)*
        * EXPERIMENTAL: adjusts contrast, saturation, etc for *better* image quality
    * HomeKit Device Setup Code *(optional)*
    * HomeKit Device Manufacturer *(optional)*
    * HomeKit Device model name *(optional)*
    * HomeKit Device model number *(optional)*
    * HomeKit Device Serial number *(optional)*
    * HomeKit Device Firmware version *(optional)*

## Build instructions ESP32

1. Make sure you have followed the `Configuration` changes above
1. Apply **esp32-camera.patch** patch:
    * `git apply --directory="components/esp32-camera" esp32-camera.patch`
    * This is only required once. 
1. Compile code `make all`
1. To prevent any effects from previous firmware (e.g. firmware crashing right at start), highly recommend to erase flash:
    * `make erase_flash`
1. Upload Upload firmware to ESP32 
    * `make flash monitor`
    * Note, ESP32 **GPIO0** pin needs to be connected to **GND** pin to enable flashing
    * To exit monitor, on mac *control+]*

    
## Add camera to HomeKit app

1. Open Home app
1. Click + sign to add accessory
1. On Add Accessory screen, click `I Don't Have a Code or Cannot Scan` button
1. The ESP camera accessory should be shown, click the icon
1. On Uncertified Accessory prompt, select *Add Anyway*
1. On Enter HomeKit Setup Code, enter setup code 
    * Default is `111-11-111`
    * Found in the HomeKit `Configuration` section above
1. Click `Continue` button
1. Select camera location, continue
1. Enter camera name, continue
1. Click Done
1. Camera ready to use


## Troubleshooting
While not HomeKit specific, good amount of troubleshooting information for flashing ESP32 devices can be found at https://randomnerdtutorials.com/esp32-cam-troubleshooting-guide/

## Other
Device runs a webserver on HTTP port **5556**
# Preparing a Control Board

TODO: Improve this with step by step instructions and screenshots

- Assemble a control board (see [v1 hardware](../hardware/v1.md) or [v2 hardware](../hardware/v2.md))
- Flash the firmware
    - See [Building & Flashing](../devs/build.md) for details
    - Install bossa cli or dfu-util and make sure the binaries (bossac or dfu-util) are in your path
    - Install python
    - Download and extract release package from [GitHub](https://github.com/MB3hel/AUVControlBoard/releases)
    - Put the board into bootloader mode
        - v1: Double press reset button quickly
        - v2: Hold boot button, press and release reset, release boot button
    - In `firmware` folder from release package run `./flash.py [v1/v2] Release`
    - This will run the flash tool
- Connect [external sensors](../hardware/sensors.md)
# AquaPack Robotics Control Board

The control board is a replacement for the Cube Orange / Pixhawk running ArduSub that was originally used on SeaWolf VIII. The control board was developed to address continuous issues with both control and sensors when using either off the shelf autopilot.

The control board is designed using development boards and breakout boards to reduce development time and simplify maintenance / modifications later. This is not a size-optimized solution (and does not need to be).

The control board generates ESC control signals (PWM), acquires and processes sensor data (using onboard IMU / IMUs, depth sensor, and sensor fusion / filtering algorithms) as well as running control loops for system motion and stability control. Additionally, an interface to the computer (Jetson) is provided to acquire sensor data and allow control of motors using high or low level methods (vectored motion or direct control of each motor’s speed).


## Firmware Development Environment

Both the [control board firmware](./firmware/) and the [hardware test](./hwtest/) program are developed using [PlatformIO](https://platformio.org/). PlatformIO is installed as an extension to [VSCode](https://code.visualstudio.com/).

The libraries for the SAMD51 and the Arm CMSIS libraries are included in this project (`lib/samd51` and `lib/cmsis`). Both of these libraries have been copied from the Atmel Studio files (`C:\Program Files (x86)\Atmel\Studio\7.0\packs\`).


## Flashing a New Board

1. [Update the U2F Bootloader](https://learn.adafruit.com/introducing-adafruit-itsybitsy-m4/update-the-uf2-bootloader)
    - Press the reset button twice to enter bootloader mode
    - Copy the downloaded bootloader `.u2f` file to the `ITSYM4BOOT` drive

2. Upload the Hardware Test Program (`hwtest`)
    - Open the `hwtest` folder in VSCode (after installing platformio extension)
    - Upload the project using the upload button on the toolbar
    - The port should be automatically detected. If not, [specify it](https://docs.platformio.org/en/latest/projectconf/section_env_upload.html) in `platformio.ini`
    - The hardware test will not begin until a serial connect is opened with the board. Open the serial monitor and make sure all tests pass. The red LED will turn off when successful, or blink on failure.

3. Upload the firmware
    - Open the `firmware` folder in VSCode (after installing platformio extension)
    - Upload the project using the upload button on the toolbar
    - The port should be automatically detected. If not, [specify it](https://docs.platformio.org/en/latest/projectconf/section_env_upload.html) in `platformio.ini`


## Control Board Usage

### Communication Protocol

TODO


### Modes of Operation

TODO


### Commands and Messages

TODO


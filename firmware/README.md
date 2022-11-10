# SW8E Control Board Firmware

## Firmware Development Environment

Both the [control board firmware](./firmware/) and the [hardware test](./hwtest/) program are developed using [PlatformIO](https://platformio.org/). PlatformIO is installed as an extension to [VSCode](https://code.visualstudio.com/).


## Flashing a New Board

1. [Update the U2F Bootloader](https://learn.adafruit.com/introducing-adafruit-itsybitsy-m4/update-the-uf2-bootloader)
    - Press the reset button twice to enter bootloader mode
    - Copy the downloaded bootloader `.u2f` file to the `ITSYM4BOOT` drive

2. Upload the firmware
    - The ItsyBitsy M4's UF2 bootloader is `sam-ba` compatible.
    - Open the `firmware` folder in VSCode (after installing platformio extension)
    - Upload the project using the upload button on the toolbar
    - The port should be automatically detected. If not, [specify it](https://docs.platformio.org/en/latest/projectconf/section_env_upload.html) in `platformio.ini`


## File Tree

- `include`: Header files
    - `cmsis`: Common Microcontroller Software Interface Standard (CMSIS) headers for Cortex M processors.
    - `samd51_dfp`: SAMD51A Device Framework Package (DFP) headers.
    - `*.h`: Firmware headers
`src`: Source files
    - `samd51_dfp`: SAMD51A Device Framework Package (DFP) sources.
    - `tinyusb`: TinyUSB library (only keeping required components for samd51). Contains both headers and source files.
    - `*.c`: Firmware sources
- `samd51g19a_flash_wbld.ld`: Linker script modified for ItsyBitsy M4 with UF2 bootloader
- `read_debug.py`: Reads debug messages from control board


## Debugging

Debugging while developing the firmware is a little difficult. The ItsyBitsy M4 has no builtin debugger. SWD pins are exposed on the header, but this requires an external debugger. Platformio supports J-LINK and Atmel-ICE debuggers for the ItsyBitsy M4. A CMSIS-DAP debugger could probably work with a [custom debug setup](https://docs.platformio.org/en/latest/plus/debug-tools/custom.html) using OpenOCD.

The main challenge is access to a debugger (J-LINK and Atmel-ICE are both expensive). Large amounts of the firmware have been developed without access to an external debugger. The following methods could be useful

- First, the red LED on pin 13 is not available. This pin is used for thruster PWM. However, you may notice it blink when the board resets. This can be useful to tell when the board resets.
- The RGB LED (dotstar) is usable though. Typically, the cmdctrl block controls the LED color. If you want to use it for debugging, comment out the call to `cmdctrl_update_led` in `main`. This LED is *very* useful for debugging because the code for it is bit banged and synchronous (blocking). Thus, it is known that the led color will be set when `dotstar_set` is called before the code continues executing.
- The USB communication with the PC can also be used, however note that if the main loop is stalled the USB will not communicate. Messages are **not** fully sent before the code continues. However, messages are queued in order. If using this for debugging, it is recommended to use the `readmsg.py` script in the demo scripts directory.
- Additionally, there are several unused GPIO pins. Pins 3 and 4 are easy to clip a lead onto. Setting these as outputs and writing them high / low when something occurs can provide information. Using a logic analyzer, this can be particularly helpful to analyze timing.

## Code Description

### Overall Development Approach

The Control Board firmware is implemented bare metal on the SAMD51 chip. No RTOS, and no use of frameworks such as ASF4 or MPLAB Harmony v3. These options were considered, but rejected.

- RTOS support for the SAMD51 is limited without using MPLAB Harmony. It also doesn't provide much of a benefit for this project. FreeRTOS could work, but peripheral drivers would still have to be developed, thus not much benefit. There is no mbedOS support. Zyphr RTOS supports the ItsyBitsy M4, but has no I2C support (thus it wasn't really considered).
- ASF4 was originally used to develop the firmware, but has several downsides. First it is required to use Atmel Start. There are some maintainability concerns with a fully web based tool longer term. Additionally, many of the higher level drivers in ASF4 are seemingly poorly tested. High level timer, watchdog, pwm, and i2c drivers were attempted at some point in the firmware. All of them were removed for the "lite" versions due to limitations or bugs in the drivers. The "lite" versions are really just a GUI to configure the peripheral the same way it would be done bare metal. At this point, ASF4 / Atmel Start provide little benefit and makes firmware development rely on an online tool. Additionally, if Atmel Start is updated, it is currently not versioned meaning compatibility could be lost in future updates (stated in official Atmel Start documentation). In other words, using ASF4 just risks having to rewrite firmware to maintain in the future.
- MPLAB Harmony v3 is an interesting framework, but has several downsides. The project setup is difficult (at best) and hard to make portable (relies on absolute paths to the framework on the system). This introduces development / maintenance complexities. Additionally, it is only compatible with MPLAB tools (proprietary XC32 compiler and Netbeans based MPLAB X IDE). The IDE is often buggy and limited in configuration features. The requirement of a proprietary compiler which may or may not have more restrictions imposed in the future is also a concern.
- The main reason to use a framework on this chip would be USB support (ASF4 and Harmony v3 both have a USB stack). However, tinyUSB works well with the SAMD51, thus the importance of this is reduced significantly. In fact, tinyUSB seems to work better than ASF4's USB stack based on limited testing.


### Code Breakdown / Architecture

TODO


## Updating Firmware via Robot

In the robot (sealed) the Control board is connected by USB to a Linux computer (Jetson, Raspberry Pi, Orange Pi, etc). When building the firmware, a `firmware.bin` file is generated in `.pio/build/adafruyit_itsybitsy_m4`. Copy this file to the robot's computer using scp / sftp.

```sh
# RUN ON DEV COMPUTER. REPLACE IP WITH ROBOT'S IP ADDRESS
scp .pio/build/adafruit_itsybitsy_m4/firmware.bin [USER]@[IP_ADDRESS]:firmware.bin
```

Then, copy `uf2conv.py` and `uf2families.json` to the robot from [this](https://github.com/microsoft/uf2) repo (in `utils` folder). 


The following command will trigger the control board to enter the bootloader (could also make this a script)

*NOTE: THIS WILL ONLY WORK IF CONTROL BOARD FIRMWARE IS INITIALIZED FULLY AND RUNNING*

```sh
# RUN VIA SSH SESSION TO ROBOT'S COMPUTER
python3
>>> from serial import Serial
>>> import time
>>> ser = Serial("/dev/ttyACM0", 1200)
>>> time.sleep(0.5)
>>> ser.close()
```

Then, `uf2conv` can be used to convert `firmware.bin` to a uf2 file and flash it to the device using the bootloader's USB MSD interface.

```sh
# RUN VIA SSH SESSION TO ROBOT'S COMPUTER
python3 uf2conv.py firmware.bin
```

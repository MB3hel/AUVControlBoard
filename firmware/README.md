# SW8E Control Board Firmware

The control board firmware is written using Microchip's Advanced Software Framework v4 (ASF4). The project is "generated" using [Atmel Start](https://start.atmel.com/). The configuration is saved as `ControlBoard.atstart` and can be edited / regenerated later. The generated project is downloaded as a makefile project. The file can be extracted as a zip. Copy the files as described below
- Copy all files in `CMSIS/Include` to `lib/cmsis/include/`
- Copy `samd51a/include` to `lib/samd51a/include` and `samd51a/gcc` to `lib/samd51a/src`. Then move all files from `lib/samd51a/src/gcc` up to `lib/samd51a/src`.
- Copy `hal/src` and `hal/include` files to `lib/asf4/src` and `lib/asf4/include` respectively
- Copy `hal/utils/src` and `hal/utils/include` files to `lib/asf4/src` and `lib/asf4/include` respectively
- Copy `hri` files to `lib/asf4/include`
- Copy files in `hpl` subfolders to `lib/asf4/src` (sources) or `lib/asf4/include` (headers)
- Copy `atmel_start.c`, `driver_init.c`, and `main.c` to `src/`
- Copy `atmel_start.h`, `atmel_start_pins.h`, and `driver_init.h` to `include/`
- Copy `config/*` to `lib/asf4/include/`


Why ASF? Why not just use CMSIS and SAMD51_DFP? A few reasons:
1. USB. Implementing a USB to UART device (USB CDC device) is a lot of work. Arduino core for samd51 does it. TinyUSB does it (using ASF) and ASF has a driver for it.
2. Clocks. The ItsyBitsy M4 Express has no external oscillators. Configuring the clock system is a pain. Atmel start makes it easy.
3. Longer term maintainability
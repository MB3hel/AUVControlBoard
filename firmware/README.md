# SW8E Control Board Firmware

The control board firmware is written using Microchip's Advanced Software Framework v4 (ASF4). The project is "generated" using [Atmel Start](https://start.atmel.com/). The configuration is saved as `ControlBoard.atstart` and can be edited / regenerated later. The generated project is downloaded as a makefile project. Use the `import_atmel_start_proj.py` script to import a modified atmel start project into this project.


Why ASF? Why not just use CMSIS and SAMD51_DFP? A few reasons:
1. USB. Implementing a USB to UART device (USB CDC device) is a lot of work. Arduino core for samd51 does it. TinyUSB does it (using ASF) and ASF has a driver for it.
2. Clocks. The ItsyBitsy M4 Express has no external oscillators. Configuring the clock system is a pain. Atmel start makes it easy.
3. Longer term maintainability
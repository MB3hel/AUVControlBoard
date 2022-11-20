# ControlBoard Firmware

- [PlatformIO](https://platformio.org/)
- [MCC Standalone](https://www.microchip.com/en-us/tools-resources/configure/mplab-code-configurator)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- Using [FreeRTOS](https://www.freertos.org/) v202112.00
- Using [TinyUSB](https://docs.tinyusb.org/en/latest/) v0.14.0
    - Note: SAMD core was adapted to work with Microchip DFP instead of Atmel DFP. The syntax changed  when Microchip acquired Atmel. The new syntax is "Microchip style" not "Atmel style". MCC will only use the newer DFP, so TinyUSB was modified.

<!--
TODO: Improve this page!
    - Describe process of using generator projects
    - Describe project organization for both targets
-->

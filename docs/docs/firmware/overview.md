# Overview

TODO: Project Structure and Build System

TODO: Generator projects

TODO: System architecture

TODO: EEPROM Emulation

- SAMD51 (CBv1)
    - Using NVMCTRL SmartEEPROM
    - Must configure size using NVMCTRL user page (requires write of fuses / config bits then device reset; XC32 compiler uses pragma config to configur this. I see no other way with GCC)
    - Note that as such, configuring fuse settings with MCC Standalone generator will do no good (it assumes XC32)
    - Linker script modified to shorten rom to avoid the data for eeprom
- STM32 (CBv2)
    - TODO

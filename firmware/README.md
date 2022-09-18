# SW8E Control Board Firmware

Firmware uses Microchip's Advanced Software Framework 4 (ASF4)

## File Tree

- `include`: Header files
    - `asf4`: All Microchip ASF4 headers (config, hal, hpl, hri, etc). Extracted from Atmel Start project.
    - `cmsis`: Common Microcontroller Software Interface Standard (CMSIS) headers for Cortex M processors Extracted from Atmel Start project.
    - `samd51a`: SAMD51A Device Framework Package (DFP) headers. Extracted from Atmel Start package.
    - `ast`: Atmel Start headers. Extracted from Atmel Start project.
    - `*.h`: Firmware headers
`src`: Source files
    - `asf4`: All Microchip ASF4 sources (hal, hpl, etc). Extracted from Atmel Start project.
    - `samd51a`: SAMD51A Device Framework Package (DFP) sources. Extracted from Atmel Start package.
    - `ast`: Atmel Start sources. Extracted from Atmel Start project.
    - `*.c`: Firmware sources
- `ControlBoard.atstart`: Atmel Start configuration file (can be loaded in Atmel Start)
- `import_atmel_start_proj.py`: Script to import Atmel Start project (`atzip` file). *Note: Adding more features to project may require modifications to this script. Inspect the downloaded zip file.*
- `pccomm_test.py`: Python script to send messages to / receive messages from the control board using the USB comm protocol. Used for development testing only.
- `samd51g19a_flash_wbld.ld`: Linker script modified for ItsyBitsy M4 with UF2 bootloader


*Note: Platformio libraries are not used for extracted cmsis, samd51a, asf4, because some source files (namely those in samd51a) must be part of executable's sources. Also just easier to keep the basic structure used by Atmel Start.*
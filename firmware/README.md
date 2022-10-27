# SW8E Control Board Firmware

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

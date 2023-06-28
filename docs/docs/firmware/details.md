# Implementation Details

TODO: EEPROM Emulation

- SAMD51 (CBv1)
    - Using NVMCTRL SmartEEPROM
    - Must configure size using NVMCTRL user page (requires write of fuses / config bits then device reset; XC32 compiler uses pragma config to configur this. I see no other way with GCC)
    - Note that as such, configuring fuse settings with MCC Standalone generator will do no good (it assumes XC32)
    - Linker script modified to shorten rom to avoid the data for eeprom
- STM32 (CBv2)
    - Using code from https://github.com/STMicroelectronics/STM32CubeF4/tree/master/Projects/STM32F411RE-Nucleo/Applications/EEPROM/EEPROM_Emulation as st_eeprom.h/c
    - Using sectors 1 and 2
    - isr_vectors must be in sector 0, most of this sector is wasted
    - Flash used by program starts at sector 3. Leaves 464k flash for use.
    - Note that since flash is split, flashing must occur in two stages (boot and main) using dfu-util



TODO: Threading model mutexes, dataflow details



TODO: Generator use and import process details



TODO: Calibration of BNO055
- EEPROM calibration status signature for valid check
- Erase only invalidates signature (potentially fewer writes)
- If calibration constants are valid, they are applied during bno055 configuration. Applying constants will prevent BNO055 from running calibration routine in bg. Applied calibration is assumed to be good by the sensor (CALIB_STAT has no relevance).
- Stored calibration commands
    - Erase triggers reset of sensor causing constants to be lost and auto calibration routine to run
    - Store triggers reset of sensor using the newly stored constants (thus no auto calibration runs)
    - Load will send the stuff from eeprom
- There are separate BNO055 commands to read its calibration status register and the values from its calibration registers (these are only valid if CALIB_STAT is 3 for sensors used and BNO055 is in config mode)
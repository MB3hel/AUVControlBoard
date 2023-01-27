# Build and Flash

*Note: Run all commands shown in the `firmware` folder of the repo.*

## Building

Install the Required Tools:

- [CMake](https://cmake.org/) (version `3.20.0` or newer)
- [Ninja](https://ninja-build.org/)
- [GNU Arm Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

*Make sure `cmake`, `ninja`, and `arm-none-eabi-gcc` are in your `PATH`.*

Build the firmware using the commands below. Replace `[preset]` with `v1` or `v2`. Replace `[config]` with `debug`, `release`, `minsizerel`, or `relwithdebinfo`.

```sh
cmake --preset=[preset]
cmake --build --preset=[preset]-[config]
```




## Flashing

| Version    | Flash Method                             | Tool Alias      | Required software                         |
| ---------- | ---------------------------------------- | --------------- | ----------------------------------------- |
| v1         | sam-ba (via bootloader)                  | `bossa`         | [BOSSA](http://www.shumatech.com/web/products/bossa) (specifically bossac / bossa-cli) |
| v1         | uf2conv (via bootloader)                 | `uf2conv`       | None |
| v2         | dfu-util (via bootloader)                | `dfu-util`      | [dfu-util](https://dfu-util.sourceforge.net/) |
| v2         | STM32CubeProgrammer DFU (via bootloader) | `stm32-dfu`     | [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) |
| v2         | STM32CubeProgrammer ST-LINK v2           | `stm32-stlink2` |[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) |

*Note: required tool (`bossac`, `dfu-util`, `STM32_Programmer_CLI`) must be in your `PATH`.*

Before flashing, the chip needs to enter its bootloader (unless using a debug probe such as the stlink2 to flash)

- *If a board is already flashed, it can be rebooted into its bootloader using the `reboot_bootloader.py` script in the `firmware` directory. Otherwise, use the hardware method described below.*

- ***v1:** Press the reset button twice quickly (double press).*

- ***v2:** Hold the BOOT button. While holding it, press and release the NRST button. Then release the boot button.*


To flash, run the `flash.py` script. It is a wrapper that will call one of the above tools

```sh
python3 flash.py [version] [config] -u [tool]
```

- `[version]` is either `v1` or `v2`
- `[config]` is the configuration you want to flash (same as configuration built: `Debug`, `Release`, `MinSizeRel`, or `RelWithDebInfo`)
- `[tool]` is one of the above upload tool aliases.

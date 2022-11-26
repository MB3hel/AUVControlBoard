# Control Board Firmware

There are multiple versions of Control Board using a different chip. Each is given a version number.
- v1: Protobaord assembly using Adafruit ItsyBitsy M4 (ATSAMD51G19A chip)
- v2: Protoboard assembly using WeAct Studio Black Pill (STM32F411CEU chip)




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
| v2         | STM32CubeProgrammer ST-LINK v2           | `stm32-stlink21 |[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) |

*Note: required tool (`bossac`, `dfu-util`, `STM32_Programmer_CLI`) must be in your `PATH`.*

To flash, run the `flash.py` script. It is a wrapper that will call one of the above tools

```sh
python3 flash.py [version] [config] -u [tool]
```

- `[version]` is either `v1` or `v2`
- `[config]` is the configuration you want to flash (same as configuration built: `Debug`, `Release`, `MinSizeRel`, or `RelWithDebInfo`)
- `[tool]` is one of the above upload tool aliases.





## Development using VSCode

Install the `C/C++` and `CMake Tools` extensions, then open this folder in VSCode. Choose one of the configure presets on the bottom bar. Then, choose a build preset. Finally, click build.

To debug, install the `Cortex-Debug` extension in VSCode. Create a `launch.json` file with a configuration in one of the following formats entry with one of the following formats. Make sure to change the binary path to match the configuration you are building!

**Debug Control Board v2 w/ ST-LINK v2**:

```json
{
    "name": "Control Board v2 w/ ST-LINK v2",
    "cwd": "${workspaceFolder}",
    "executable": "./build/v2/Debug/ControlBoard.elf",
    "request": "launch",
    "type": "cortex-debug",
    "runToEntryPoint": "main",
    "servertype": "openocd",
    "configFiles": [
        "interface/stlink-v2.cfg",
        "target/stm32f4x.cfg"
    ]
}
```





## Generator Projects

Each version of ControlBoard has an associated "Generator" project. These are projects used with the chip manufacturer's tools to generate startup / configuration / library code for the chip.

*Note: If you are just building the firmware you do not need to understand the generator projects. The necessary portions of each project are copied to the `thirdparty` folder. These are used for building. The generator projects are only used if something that was generated needs to be changed (or something new needs to be generated).*

### Why Generators

The decision to use generator projects comes down to the following

1. There are multiple versions of Control Board with multiple chips. Using generators to create chip-specific initialization code reduces the chip-specific versions of the firmware that must be written / maintained.

2. The generators are often required to use the manufacturer's HAL (or make using the HAL much easier). Use of the manufacturer's HAL / libraries reduces development time and makes maintenance easier (especially for those less familiar with the codebase).

3. Having a GUI tool (which these generators usually are) to configure the system is an easy way to quickly understand the architecture of the system (clocks, peripheral use, pinout, etc).

### Running the Generator(s)

#### Control Board v1

Control Board v1 uses an Adafruit ItsyBitsy M4 board (Microchip ATSAMD51G19A chip). The generator used for this project is [MCC Standalone](https://www.microchip.com/en-us/tools-resources/configure/mplab-code-configurator). This tool is available for download on Windows, macOS, and Linux. Once installed
- Launch the application
- Make sure the Harmony Content Path is set in `Tools` > `Options`. This should only need to be done once per computer. Recommended path is `~/.mcc/harmony/v3`.
- `File` > `Load Configuration`
- Choose `generator_projects/ControlBoard_v1/firmware/ControlBoard_v1/ControlBoard_v1.mc3`
- You will likely be prompted to install MPLAB Harmony content. Install it.
- Click "Generate" in the top left.

#### Control Board v2

Control Board v2 uses a WeAct Studio Black Pill board (STMicro STM32F411CEU chip). The generator used for this project is [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html). This tool is available for download on Windows, macOS, and Linux. Once installed
- Launch the application
- `File` > `Load Project`
- Choose `generator_projects/ControlBoard_v2/ControlBoard_v2.ioc`
- Install any required packs (as prompted)
- Click "Generate Code" in the top right.


### Importing from Generators

After running the generator (as described above) the generated code must be imported to the project. The import process is mostly just copying generated files, however some files are modified slightly. The import process is handled by the `import_from_generator.py` script. If additional components are added in the generator projects, this script may need to be modified to import additional components. When this script is run, it will prompt a version of control board to import generated code for. This script must be run after each time the generator project is modified and code is re-generated.

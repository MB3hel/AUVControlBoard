# Firmware Development

## Build and Flash

Building and flashing the firmware should follow the same process described [here](./build.md).


## Debugging

OpenOCD config files exist in `tools/debug`. The following configurations exist

| ControlBoard Version | Debugger / Debug Probe | Config Name                  |
| -------------------- | ---------------------- | ---------------------------- |
| v1                   | CMSIS-DAP&ast;         | `cb_v1_via_cmsisdap.cfg`     |
| v2                   | ST-LINK v2             | `cb_v2_via_stlink2.cfg`      |

&ast;There is a firmware for a Raspberry Pi Pico to turn it into a CMSIS-DAP debugger. This is a variant of the Picoprobe firmware.

*Note that [OpenOCD](https://openocd.org/) must be installed and in the `PATH`.*


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

Control Board v1 uses an Adafruit ItsyBitsy M4 board (Microchip ATSAMD51G19A chip). The generator used for this project is [MCC Standalone](https://www.microchip.com/en-us/tools-resources/configure/mplab-code-configurator). This tool is available for download on Windows, macOS, and Linux. Tested with v5.2.1 of MCC Standalone.

Once installed

- Launch the application
- Make sure the Harmony Content Path is set in `Tools` > `Options`. This should only need to be done once per computer. Recommended path is `~/.mcc/harmony/v3`.
- `File` > `Load Configuration`
- Choose `generator_projects/ControlBoard_v1/firmware/ControlBoard_v1/ControlBoard_v1.mc3`
- You will likely be prompted to install MPLAB Harmony content. Install it.
- Click "Generate" in the top left.

#### Control Board v2

Control Board v2 uses a WeAct Studio Black Pill board (STMicro STM32F411CEU chip). The generator used for this project is [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html). This tool is available for download on Windows, macOS, and Linux. Tested with v6.6.1 of STM32CubeMX.

Once installed

- Launch the application
- `File` > `Load Project`
- Choose `generator_projects/ControlBoard_v2/ControlBoard_v2.ioc`
- Install any required packs (as prompted)
- Click "Generate Code" in the top right.


### Importing from Generators

After running the generator (as described above) the generated code must be imported to the project. The import process is mostly just copying generated files, however some files are modified slightly. The import process is handled by the `import_from_generator.py` script. If additional components are added in the generator projects, this script may need to be modified to import additional components. When this script is run, it will prompt a version of control board to import generated code for. This script must be run after each time the generator project is modified and code is re-generated.


## Development using VSCode

Install the `C/C++` and `CMake Tools` extensions, then open this folder in VSCode. Choose one of the configure presets on the bottom bar. Then, choose a build preset. Finally, click build.

To debug, install the `Cortex-Debug` extension in VSCode. Copy `tools/debug/launch.json` to `.vscode`. *MAKE SURE TO BUILD DEBUG CONFIG BEFORE LAUNCHING DEBUG SESSION. IT WILL NOT BUILD AUTOMATICALLY.*
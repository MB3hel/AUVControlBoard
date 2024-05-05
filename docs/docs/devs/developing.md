# Firmware Development

## VSCode Setup

Often, it is useful to have some sort of code editor / IDE setup. The project is already setup to work easily with [Visual Studio Code](https://code.visualstudio.com/) (VSCode), provided a few extensions are installed.

After installing VSCode

- Install the `C/C++` and `CMake Tools`, and `Cortex-Debug` extensions for working with the firmware (`firmware/` subfolder)
- Install the `Python` extension for working with the interface scripts (`iface/` subfolder)


To use, just open one of the folders (eg `firmware/` or `iface/`) in VSCode. The useful extensions should automatically load when you open the folders (you may have to open a source code file in some cases).

For debugging, a `launch.json` file is included in the `firmware/` project (in `.vscode` subfolder). This defines debug configurations both for firmware running on physical hardware and for the supported SimCB configurations.

*Note: Launching a debug session from VSCode will not build the firmware. Make sure to build the "Debug" configuration before launching!*


## Debugging Firmware

### Debugging on Physical System

This section discusses debugging of the firmware running on actual hardware (Control Board v1 or v2). Debugging firmware running on a microcontroller requires a "debug probe". The supported probes for each version of the control board are listed in the table below.

| ControlBoard Version | Debugger / Debug Probe | Config Name in `tools/debug` |
| -------------------- | ---------------------- | ---------------------------- |
| v1                   | CMSIS-DAP&ast;         | `cb_v1_via_cmsisdap.cfg`     |
| v2                   | ST-LINK v2             | `cb_v2_via_stlink2.cfg`      |

&ast;The PicoProbe firmware can be used to turn a low cost Raspberry Pi Pico board into a CMSIS-DAP compliant debug probe.

You must wire the debug probe to the microcontroller on the control board correctly. Both v1 and v2 use a SWD debug interface. Examples of wiring the debug probes to each are shown below.

TODO: Wiring diagrams for v1 and v2

***Note: NEVER power the system from both the debug probe and USB at the same time. Generally, power the system over USB and do not connect the power (3.3V) from the debug probe. Signals and GND must be connected from the debug probe.***


Make sure that [OpenOCD](https://openocd.org/) must be installed and in the `PATH`. 

On Linux systems, you will likely also need to install `gdb-multiarch` and symlink it to `arm-none-eabi-gdb`. On ubuntu, this is done with the following commands

```sh
sudo apt install gdb-multiarch
sudo ln -s `which gdb-multiarch` /usr/local/bin/arm-none-eabi-gdb
```

Then, use VSCode to run the correct debug session for the hardware version and debug probe you are using. Alternatively, you may use openocd and gdb directly if you are not using vscode. The config files listed in the table above are just OpenOCD configs.


### Debugging SimCB

For windows, you can attach the windows debugger without any issues / modifications.

For macos / linux (using FreeRTOS posix port) the debugger must have some additional configuration to avoid impacting signals used by the port to facilitate context switches.

LLDB (macOS):

```
process handle SIGUSR1 --notify false --pass true --stop false
process handle SIGALRM --notify false --pass true --stop false
```

GDB (Linux):

```
handle SIGUSR1 nostop noignore noprint
handle SIGALRM nostop noignore noprint
```


## Generator Projects

*Note: All paths in this section are relative to the `firmware/` subdirectory.*


Each version of ControlBoard has an associated "Generator" project. These are projects used with the chip manufacturer's tools to generate startup / configuration / library code for the chip.

You will only need to modify the generator projects to alter configuration of the chip / components of included libraries from these tools. The firmware copies code generated from these tools into the `thridparty` folder, thus you don't need the generators to modify the firmware, unless you are modifying generated components.

### Running the Generator(s)

#### Control Board v1

Control Board v1 uses an Adafruit ItsyBitsy M4 board (Microchip ATSAMD51G19A chip). The generator used for this project is [MCC Standalone](https://www.microchip.com/en-us/tools-resources/configure/mplab-code-configurator). This tool is available for download on Windows, macOS, and Linux. Tested with v5.2.1 of MCC Standalone.

Once installed

- Launch the application
- Make sure the Harmony Content Path is set in `Tools` > `Options`. This should only need to be done once per computer. Recommended path is `~/.mcc/harmony/v3`. On windows, use `C:\Users\[YOUR_USERNAME]\.mcc\harmony\v3` (replace `[YOUR_USERNAME]`).
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

To run the importer, run `python3 import_from_generator.py` then choose which project your are importing.


## Firmware Development using the Simulator

The [simulator](https://github.com/MB3hel/GodotAUVSim) can be very helpful for developing or debugging firmware. The simulator is capable of connecting to a real control board or SimCB binaries. Thus the firmware can be tested in a simulation environment to validate math or behavior of a vehicle. Testing with the simulator can also be done while the control board firmware is running with a debugger attached (true for both a real control board and SimCB).

Testing firmware on a physical control board (v1, v2) with the simulator is fairly simple. It is assumed that you will want the debugger running too:

- Connect the control board to your PC via USB
- Connect a debug probe to the control board (do **NOT** connect power)
- Build and run the firmware under debugger
- In the simulator, choose the UART port for the control board
- Interface scripts can be run as usual using `launch.py`, but add the `-s` flag so it will connect to the simulator instead of the physical control board.


If you want to debug SimCB using the simulated environment

- TODO: Requires more work on simulator UI! NYI!


There are a few things to know about running under simulation (these are true when the firmware is hijacked by simulator)

- Sensor drivers are disabled while under simulator control. Additionally, the active sensors will always be reported as the "SIM" sensors. This is true even if a real sensor is connected to a physical control board under simulation.
- When controlled by the simulator, the control board will not generate PWM signals on thruster pins. The thruster pins will maintain a pulse corresponding to no motion (or no pulse at all if no thruster pwm parameters have been applied).


## Developing Python Interface Scripts

TODO

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

*Note: required tool (`bossac`, `dfu-util`, `STM32_Programmer_CLI`) must be in your `PATH`.*

Before flashing, the chip needs to enter its bootloader (unless using a debug probe such as the stlink2 to flash)

- *If a board is already flashed, it can be rebooted into its bootloader using the `reboot_bootloader.py` script in the `firmware` directory. Otherwise, use the hardware method described below.*

- ***v1:** Press the reset button twice quickly (double press).*

- ***v2:** Hold the BOOT button. While holding it, press and release the NRST button. Then release the boot button.*

*Note: Sometimes reboot to bootloader mode "fails". On v1, this usually means it fails to attach USB (LED remains red not green.) On v2 this usually means it doesn't show up as a USB device. In either case, just try to enter again using the same button combination above.*

To flash, run the `flash.py` script. It is a wrapper that will call one of the above tools

```sh
python3 flash.py [version] [config] -u [tool]
```

- `[version]` is either `v1` or `v2`
- `[config]` is the configuration you want to flash (same as configuration built: `Debug`, `Release`, `MinSizeRel`, or `RelWithDebInfo`)
- `[tool]` is one of the above upload tool aliases.


## Flashing Remotely

Sometimes, it is useful to flash firmware without connecting directly to the control board. Typically, this is done in-system where the embedded computer using the control board (Jetson, Raspberry Pi, etc) is used to flash the control board without gaining physical access to the control board. Instead an ssh connection to the remote computer is used.

There are a few requirements to be able to flash remotely

- The control board must already be running some version of the firmware. This is necessary to be able to enter bootloader mode without access to the buttons on the board.
- The remote computer must have a flash tool installed. For v1 this should be `bossac` and for v2 this should be `dfu-util`. These are available as packages for most Linux distributions (`bossa-cli` and `dfu-util` respectively for Debian and Ubuntu based systems).
- You must have ssh (and by extension scp) access to the remote system (typically via ethernet tether)

On the build computer (laptop, etc) build the firmware as described above. Then, login to the remote system via ssh and create a directory to hold control board flash stuff (name can be changed as desired)

```sh
# Run on remote computer (via ssh)
cd ~
mkdir cboard-flash
```

Then on the build computer, use scp to copy the `flash.py` and `reboot_bootloader.py` scripts to this folder

```sh
# Run on build laptop
scp firmware/flash.py user@remote_ip:cboard-flash/
scp firmware/reboot_bootloader.py user@remote_ip:cboard-flash/
```

Next copy the `build` folder. You can just copy the binaries themselves, but the folder hierarchy must be maintained.

```sh
# Run on remote computer (via ssh)
# Delete old build folder first
rm -r ~/cboard-flash/build

# Run on build laptop
scp -r firmware/build user@remote_ip:cboard-flash/
```

Finally, reboot the control board to bootloader and flash

```sh
# Run on remote computer (via ssh)
./reboot_bootloader.py [port]
./flash.py [version] [config] -p [port]
```

*Note: Sometimes reboot to bootloader mode "fails". On v1, this usually means it fails to attach USB (LED remains red not green.) On v2 this usually means it doesn't show up as a USB device. In either case, you will loose USB communication to the board. In this case, a power cycle is required to "fix" the board before trying to enter the bootloader again. While inconvenient, it is still usually easier to power cycle the vehicle than to unseal it.*

# SW8E Control Board Firmware

TODO: Description


## Libraries & Dependencies

Two libraries are required for code targeting the SAMD51 chip on the Adafruit ItsyBitsy M4 board.

- SAMD51 libraries (from Atmel CMSIS libraries for SAMD51)
- Arm CMSIS libraries (from Arm)

Both of these libraries have been copied from the Atmel Studio files (`C:\Program Files (x86)\Atmel\Studio\7.0\packs\`). These have been integrated as libraries in this project (`lib/samd51` and `lib/cmsis`).

Finally, when using gcc (as platformio does for the `atmelsam` platform) one of the included `ld` files must be selected using the `board_build.ldscript` parameter in the `platformio.ini` file. The available `ld` files are in the `lib/samd51/src/` folder.


## Communication Protocol

TODO


## Modes of Operation

TODO


## Commands and Messages

TODO


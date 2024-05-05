# Build and Run SimCB

Install the Required Tools:

- [CMake](https://cmake.org/) (version `3.20.0` or newer)
- [Ninja](https://ninja-build.org/)
- C Toolchain for your system
    - MSVC on windows
    - Apple Clang on macOS
    - GNU Toolchain (GCC) on Linux

*Make sure `cmake` and `ninja` are in your `PATH`.*

Build the firmware using the commands below. Replace`[preset]` with `simcb-win`, `simcb-macos`, or `simcb-linux`. Replace `[config]` with `debug`, `release`, `minsizerel`, or `relwithdebinfo`.

```sh
cmake --preset=[preset]
cmake --build --preset=[preset]-[config]
```

The SimCB binary will exist in `build/[preset]/[config]/SimCB[.exe]`
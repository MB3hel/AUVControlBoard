# Control Board v1 Hardware

## Components

- Adafruit ItsyBitsy M4 Express Development Board
    - Cortex M4 CPU clocked at 120MHz. Includes hardware FPU.
- Adafruit BNO055 Breakout
    - 9DOF IMU (Gyro + Accel + Mag) with good quality sensors. Includes an on-chip processor running sensor fusion algorithms.
    - *Note that the Stemma QT version has a different pinout. Keep this in mind if using this variant of the breakout board.*
- ProtoBoard (24 by 18) **not stripboard** (rows should not be connected)
- Solid core wire (22 AWG)
- Female pin headers (can carefully cut if necessary)
    - Two 1x14 headers
    - One 1x6 header
    - One 1x4 header
- Male pin headers (can cut)
    - Two 1x8 headers
    - One 1x4 header
- Two 10K resistors (through hole, 1/4 W)
- ~~Adafruit BMP280 Breakout~~
    - **NOT NEEDED AND NOT SUPPORTED IN FIRMWARE. RECOMMENDED TO OMIT AS IT ADDS UNNECESSARY COST.**
    - ~~Pressure and temperature sensor~~
    - ~~Not likely to be used if on-chip calibrations of BNO055 are used, however some sensor calibration algorithms take into account temperature and pressure. This is present in case this information is needed at a later date.~~
    - ~~Can also be used to detect abnormal conditions (eg over temperature)~~


## Assembly

Prototype is assembled on protoboard. Final assembled prototype and pinout is shown below.

![](.//prototype_assembled_labeled.png)

The default coordinate system as defined by the IMU is shown below. Note that this is a right hand coordinate system. The red arrows define axes. Rotation about these axes is in the right hand direction (indicated by green arrows). Notice that the green arrows are on top of the red axis arrows, thus a "left to right" arrow is left to right across the top of the axis.

The axis configuration can be changed to match any mounting position of the control board on a robot. See the BNO055 datasheet pages 26 and 27 for more information (datasheet included in project `references` folder).

![](.//prototype_axis.png)


### Assembly Instructions

1. Solder header strips in the positions shown below. The female headers avoid soldering breakouts / dev boards directly to protoboard which allows easily replacing components if needed (or reusing them for other purposes later). After soldering, breakouts can be populated to make identifying pins easier.

![](.//fritzing_header_pos.png)

![](.//prototype_headers.png)

2. Wire SDA and SCL wires from MCU board to each sensor as shown below (connect the two pins shown in each row).

| Pin 1                      | Pin 2                    |
| -------------------------- | ------------------------ |
| ItsyBitsy SDA              | BNO055 SDA               |
| ItsyBitsy SDA              | BMP280 SDI               |
| ItsyBitsy SDA              | Depth SDA                |
| ItsyBitsy SCL              | BNO055 SCL               |
| ItsyBitsy SCL              | BMP280 SCK               |
| ItsyBitsy SCL              | Depth SCL                |


3. Wire power and ground to each sensor as shown below. Power comes from USB on MCU dev board through builtin regulator.

| Pin 1                      | Pin 2                    |
| -------------------------- | ------------------------ |
| ItsyBitsy 3V               | BNO055 VIN               |
| ItsyBitsy 3V               | BMP280 VIN               |
| ItsyBitsy 3V               | Depth VCC                |
| ItsyBitsy GND              | BNO055 GND               |
| ItsyBitsy GND              | BMP280 GND               |
| ItsyBitsy GND              | Depth GND                |

4. Wire PWM signal header pins to GPIO pins on the dev board

| Pin 1                      | Pin 2                    |
| -------------------------- | ------------------------ |
| ItsyBitsy 13               | PWM Signal 1             |
| ItsyBitsy 12               | PWM Signal 2             |
| ItsyBitsy 11               | PWM Signal 3             |
| ItsyBitsy 10               | PWM Signal 4             |
| ItsyBitsy 9                | PWM Signal 5             |
| ItsyBitsy 7                | PWM Signal 6             |
| ItsyBitsy 1                | PWM Signal 7             |
| ItsyBitsy 0                | PWM Signal 8             |

![](.//prototype_assembled.png)

5. While not shown in pictures above, it is recommended to add a 10K pullup resistor on both the SDA and SCL lines. For each line (SDA and SCL) connect one edge of the resistor to the line (SDA or SCL) and the other side to 3V. These are not strictly required as the IMU includes a level shifter on these pins with 10K pullups, but putting additional ones in parallel helps if power rail on IMU becomes noisy (usually due to touching the pins) or if the IMU is not populated. *This significantly reduces the number of I2C "glitches" that happened during development, thus is highly encouraged.*

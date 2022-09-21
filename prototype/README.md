# AquaPack Robotics Control Board Prototype

## Components

- Adafruit ItsyBitsy M4 Express Development Board
    - Cortex M4 CPU capable of 32-bit integer math and includes hardware multiplier and divider
    - Cortext M4 CPU has a hardware floating point unit
    - The above two features help reduce software complexity making the firmware easier to maintain, debug, and develop
- Adafruit BNO055 Breakout
    - 9DOF IMU (Gyro + Accel + Mag) with good quality sensors
    - Includes an on-chip processor running sensor fusion algorithms and can provide useful information such as a gravity vector or an orientation in 3D space.
    - These features remove the need to develop sensor calibration, fusion, and interpretation algorithms in the control board firmware (simplifies firmware and accelerates development time)
    - Can also provide raw sensor data meaning custom algorithms are still possible if deemed necessary / beneficial at a later time
- Adafruit BMP280 Breakout
    - Pressure and temperature sensor
    - Not likely to be used if on-chip calibrations of BNO055 are used, however some sensor calibration algorithms take into account temperature and pressure. This is present in case this information is needed at a later date.
    - Can also be used to detect abnormal conditions (eg over temperature)


## Assembly

Prototype is assembled on protoboard. Final assembled prototype is shown below.

![](.//prototype_assembled.png)


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

![](.//prototype_i2c.png)

3. Wire power and ground to each sensor as shown below. Power comes from USB on MCU dev board through builtin regulator.

| Pin 1                      | Pin 2                    |
| -------------------------- | ------------------------ |
| ItsyBitsy 3V               | BNO055 VIN               |
| ItsyBitsy 3V               | BMP280 VIN               |
| ItsyBitsy 3V               | Depth VCC                |
| ItsyBitsy GND              | BNO055 GND               |
| ItsyBitsy GND              | BMP280 GND               |
| ItsyBitsy GND              | Depth GND                |

![](.//prototype_power1.png)

![](.//prototype_power2.png)

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

![](.//prototype_pwm1.png)

![](.//prototype_pwm2.png)

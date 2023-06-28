# Sensor Calibration

## BNO055 (IMU) Calibration

### General Information

The BNO055 IMU used on the control board supports calibration via the following values (referred to as calibration constants)

- Accelerometer Offset for each axis (x, y, z)
- Accelerometer Radius
- Gyroscope Offset for each axis (x, y, z)

Unless a calibration is written to the sensor during reset, the sensor will run a calibration routine in the background. By moving the device as described in the sensor's datasheet, this automatic calibration routine can produce a set of calibration constants resulting in improved sensor performance.

However, each time the sensor is reset (such as when the control board is power cycled) this calibration is lost. Thus, it would be required to repeat the motions required to calibrate the sensor each time the vehicle is powered on. This is often impractical and unnecessary.

To address this, the control board is capable of storing a set of BNO055 calibration constants and automatically applying them on power up. When this is done, the BNO055 does **not** run it's calibration routine in the background. The applied calibration is assumed to be valid.

This creates a scenario where there are potentially two sets of calibration constants: the ones read from the IMU and the ones saved on the control board. These two sets will be identical, unless no constants have been saved to the control board. Also note that erasing a set of constants stored to the control board will reset the BNO055, thus allowing its automatic calibration routine to run again.

The calibration constants saved on the control board are referred to as "Stored Calibration Constants" whereas the ones from the sensor are referred to as the "BNO055 Calibration Constants".


### Calibration Procedure

**Use the `example/bno055_calibration.py` interface script. This script can be run to perform calibration (guided calibration).**

The script referenced above follows this same procedure:

1. Erase any stored calibration constants. This will cause the BNO055 to run its calibration routine.
2. Perform the required motions for calibration as described in the sensor's datasheet. For the gyroscope, the device must sit stationary for several seconds. For the accelerometer, the device must be slowly moved to 6 orientations (axis-aligned) holding each orientation for several seconds.
3. While performing the motions, the control board can be asked to report the BNO055's calibration status. This status is the value of the BNO055's `CALIB_STAT` register and can be used to determine when the relevant sensors (accelerometer and gyroscope) are fully calibrated (status for each sensor is 3).
4. Once the calibration is good (status for each sensor is 3), the control board can be asked to report the BNO055's calibration constants. This will read the calibration constants from the IMU's registers.
5. Finally, the constants reported in the previous step can be stored to the control board as stored calibration constants. These will be applied to the BNO055 each time the control board starts; thus the BNO055's calibration routine will not run and the calibration status will be all 3's (the saved calibration is assumed to be good).
6. The stored calibration constants can be erased to re-calibrate the sensor. Erasing calibration constants will reset the BNO055 and its calibration routine will run again.


### When to Re-Calibrate

You should always recalibrate if you change the physical sensor in use on the control board (swap a different BNO055) or if you are using a different control board. Calibration constants will vary between sensors, thus constants from one control board cannot be used on another. Likewise, if you change the sensor on your control board the old constants will no longer be valid.

Additionally, significant operating environment changes (pressure, temperature, elevation, etc) can cause enough of a change in sensor behavior to require re-calibration. Similarly, a change of the position of the sensor in the vehicle could require re-calibration. It is recommended to recalibrate if any such changes seem to result in degredation of sensor performance.

# AquaPack Robotics Control Board

The control board is a coprocessor used by SeaWolf VIII's computer to control the robot's motion. It can be considered a complex motor controller, but can also provide sensor data to the computer for use in mission software.


## Repository Structure

- `docs`: Documentation for users of the control board. This is **not** firmware developer documentation.
- `firmware`: PlatformIO project and source code for ControlBoard firmware.
- `hardware`: Documentation & design files for hardware control board. Includes hardware design files and assembly instructions as applicable.
- `math`: Description of the math used for 6-DOF vectored motor control as well as a numpy script demoing the math.
- `references`: Various links and pdfs providing information about components used on the control board. Intended for firmware developers.

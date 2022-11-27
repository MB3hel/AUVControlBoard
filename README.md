# AUV Control Board

Motion controller for Autonomous Underwater Vehicles (AUVs) supporting motion in 6 degrees of freedom. Designed for vehicles with fixed position thrusters.

The Control Board is a coprocessor designed to assist a main computer (typically an embedded Linux computer) control the vehicle.


## Repository Structure

- `docs`: Documentation for users of the control board. This is **not** firmware developer documentation.
- `firmware`: PlatformIO project and source code for ControlBoard firmware.
- `hardware`: Documentation & design files for hardware control board. Includes hardware design files and assembly instructions as applicable.
- `math`: Description of the math used for 6-DOF vectored motor control as well as a numpy script demoing the math.
- `references`: Various links and pdfs providing information about components used on the control board. Intended for firmware developers.

# AquaPack Robotics Control Board

The control board is a coprocessor used by SeaWolf VIII's computer to control the robot's motion. It can be considered a complex motor controller, but can also provide sensor data to the computer for use in mission software.


## Repository Structure

- `firmware`: PlatformIO project and source code for the control board firmware. The readme has firmware developer oriented documentation.
- `docs`: Markdown documentation files for control board. This is end-user facing documentation.
- `math`: Documentation of the math used for motor control, as well as a demo in numpy.
- `prototype`: Assembly instructions for control board prototype
- `references`: Datasheets and documentation on relevant components / sensors used on Control Board.
- `scripts`: Python scripts that demo interfacing with the control board

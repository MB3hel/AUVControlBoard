# AUV Control Board

Motion controller for Autonomous Underwater Vehicles (AUVs) supporting motion in 6 degrees of freedom. Designed for vehicles with fixed position thrusters.

The Control Board is a coprocessor designed to assist a main computer (typically an embedded Linux computer) control the vehicle.


## Repository Structure

- `iface`: Contains reference implementation to communicate with and use control board. Written in python.
- `docs`: Documentation for users of the control board. This is a mkdocs project.
- `firmware`: PlatformIO project and source code for ControlBoard firmware.
- `references`: Various links and pdfs providing information about components and libraries used on the control board. Intended for firmware developers.


## View Documentation

```sh
cd docs
python3 -m pip install mkdocs
mkdocs serve
# Open http://localhost:8000/ in a web browser
```

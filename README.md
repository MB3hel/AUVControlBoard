# AUV Control Board

Motion controller for Autonomous Underwater Vehicles (AUVs) supporting motion in 6 degrees of freedom. Designed for vehicles with fixed position thrusters.

The Control Board is a coprocessor designed to assist a main computer (typically an embedded Linux computer) control the vehicle.


## Repository Structure

- `docs`: Documentation for users of the control board. This is a mkdocs project.
- `firmware`: PlatformIO project and source code for ControlBoard firmware.
- `iface`: Contains reference implementation to communicate with and use control board. Written in python.
- `references`: Various links and pdfs providing information about components and libraries used on the control board. Intended for firmware developers.

## License

- The firmware (`firmware` directory) is licensed under the GPU General Public License version 3.0 or later (GPL-3.0-or-later). Note that third party code contained in the firmware directory (`firmware/thirdparty`) is **not** covered by this license. Third party code is distributed under the terms of the software's own license. License files are included for such third party software.
- The documentation (`docs` directory) is currently all rights reserved
- The interface scripts (`iface` directory) is currently all rights reserved
- The reference information (`references`) is a collection of various resources from various sources. These are all publicly accessible (and downloadable) resources (at time of writing), but are not owned by or licensed by this project in any way. The license terms of these resources depend on the specific resource.


## View Documentation

```sh
cd docs
python3 -m pip install mkdocs
mkdocs serve
# Open http://localhost:8000/ in a web browser
```


from vehicle import Vehicle, register_vehicle, set_default_vehicle
from typing import List, Tuple
from control_board import ControlBoard


class MyVehicle(Vehicle):
    @property
    def motor_matrix(self) -> ControlBoard.MotorMatrix:
        return ControlBoard.MotorMatrix()

    @property
    def tinv(self) -> List[bool]:
        return [False, False, False, False, False, False, False, False]

    @property
    def reldof(self) -> List[float]:
        return [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

    @property
    def bno055_axis_config(self) -> ControlBoard.BNO055Axis:
        return ControlBoard.BNO055Axis.P1

    @property
    def xrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        return [0, 0, 0, 0, False]

    @property
    def yrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        return [0, 0, 0, 0, False]
    
    @property
    def zrot_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        return [0, 0, 0, 0, False]

    @property
    def depth_pid_tuning(self) -> Tuple[float, float, float, float, bool]:
        return [0, 0, 0, 0, False]


class MyVehicleSim(MyVehicle):
    # Override only the methods you need to here
    # Often only PID tunings (if anything) will need to change
    # Depends on how well your vehicle is modeled in the simulator
    pass


register_vehicle("my_vehicle", MyVehicle(), MyVehicleSim())

# Optional (changes vehicle used by launch.py when no vehicle is specified)
set_default_vehicle("my_vehicle")
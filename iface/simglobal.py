
from control_board import ControlBoard, Simulator
import time


s = Simulator()
cb = s.control_board
s.reset_sim()
s.set_robot_rot(*s.euler_to_quat(90, 0, 0))
time.sleep(1)
res = cb.set_global(0.0, 0.0, 0.3, 0.2, 0.0, 0.0)
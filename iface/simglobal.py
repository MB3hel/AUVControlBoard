
from control_board import ControlBoard, Simulator
import time
import threading


s = Simulator(cb_debug=True)
cb = s.control_board
s.reset_sim()
s.set_robot_rot(*s.euler_to_quat(90, 0, 0))
time.sleep(1)
res = cb.set_global(0.0, 0.0, 0.3, 0.2, 0.0, 0.0)
t_stop = False
def do_feed():
    while not t_stop:
        cb.feed_motor_watchdog()
        time.sleep(0.25)
t = threading.Thread(daemon=True, target=do_feed)
t.start()
print("Press enter to stop...", end="")
input("")
t_stop = True
t.join()
cb.set_global(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
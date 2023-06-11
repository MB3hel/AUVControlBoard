
if __name__ == "__main__":
    print("Do not run this script directly. Use launch.py to run it.")
    exit(1)

from control_board import ControlBoard, Simulator


def run(cb: ControlBoard, s: Simulator) -> int:
    # cb will always be an instantiated and connected control board instance
    # s will be None unless running under simulation, in which case it is a simulator instance
    # Return an error code from this function (launch.py will exit with the error code returned here)
    return 0

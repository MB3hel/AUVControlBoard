
from control_board import ControlBoard
import sys
import time
import traceback


if __name__ == "__main__":
    port = "/dev/ttyACM0"
    if len(sys.argv) > 1:
        port = sys.argv[1]
    print("Port: {}".format(port))

    cb = None
    try:
        cb = ControlBoard(port, debug=True)
    except:
        traceback.print_exc()
        print("Failed to open communication with control board.")
        exit(1)
    
    print("Setting RAW speed...")
    try:
        cb.set_raw([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    except:
        traceback.print_exc()
        exit(1)
    print("RAW speed set.")
import controlboard
from controlboard import ControlBoard
import traceback
import time


if __name__ == "__main__":
    controlboard.debug_prints = False
    controlboard.log_rx = True

    # Connect to and configure control board
    try:
        print("Starting...", flush=True)
        try:
            cb = ControlBoard("/dev/ttyACM0")
        except Exception as e:
            print("Fail.", flush=True)
            print(str(e))
            # traceback.print_exc()
            exit(1)
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("")
        print("Interrupted by user")
        exit(1)

    print("")
    exit(0)
    

import controlboard
from controlboard import ControlBoard
import traceback
import time


if __name__ == "__main__":
    controlboard.debug_prints = False

    # Connect to and configure control board
    try:
        print("Connecting to control board...", end="", flush=True)
        try:
            cb = ControlBoard("/dev/ttyACM0")
            print("Done.", flush=True)
        except Exception as e:
            print("Fail.", flush=True)
            print(str(e))
            # traceback.print_exc()
            exit(1)
    except KeyboardInterrupt:
        print("")
        print("Interrupted by user")
        exit(1)

    print("")
    
    # Show the data
    start_time = time.time()
    try:
        while True:
            if(cb.comm_lost):
                exit(2)
            quat = cb.get_orientation_quat()
            grav = cb.get_gravity_vector()
            print("({:05d}) QW: {:.4f}\tQX: {:.4f}\tQY: {:.4f}\tQZ: {:.4f}\tGX: {:.4f}\tGY: {:.4f}\t GZ: {:.4f}".format(
                int((time.time() - start_time)*1000),
                quat.w, quat.x, quat.y, quat.z,
                grav.x, grav.y, grav.z
            ))
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("")
        print("Interrupted by user")
        exit(1)
    exit(0)
    
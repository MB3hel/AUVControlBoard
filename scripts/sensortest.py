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
            cb.read_sensors()
            pry = cb.get_orientation()
            grav = cb.get_gravity_vector()
            depth = cb.get_depth()
            print("({:05d}) P: {:.4f}\tR: {:.4f}\tY: {:.4f}\tGX: {:.4f}\tGY: {:.4f}\tGZ: {:.4f}\tD: {:.4f}".format(
                int((time.time() - start_time)*1000),
                pry.pitch, pry.roll, pry.yaw,
                grav.x, grav.y, grav.z,
                depth
            ))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("")
        print("Interrupted by user")
        exit(1)
    exit(0)
    

from controlboard import ControlBoard
import traceback
import time


if __name__ == "__main__":
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
    try:
        while True:
            quat = cb.get_orientation_quat()
            print("W: {:.4f}\tX: {:.4f}\tY: {:.4f}\tZ: {:.4f}\t".format(
                quat.w, quat.x, quat.y, quat.z
            ))
    except KeyboardInterrupt:
        print("")
        print("Interrupted by user")
        exit(1)
    exit(0)
    
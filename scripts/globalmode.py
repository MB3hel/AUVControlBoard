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

        print("Setting mode to GLOBAL...", end="", flush=True)
        if cb.set_mode(ControlBoard.Mode.GLOBAL):
            print("Done.", flush=True)
        else:
            print("Fail.", flush=True)
            exit(1)

        print("Setting motor inversions...", end="", flush=True)
        if cb.set_inverted(True, True, False, False, True, False, False, True):
            print("Done.", flush=True)
        else:
            print("Fail.", flush=True)
            exit(1)

        #           x       y       z     pitch    roll    yaw        
        target = [  0,      0.3,    0,      0,      0,      0   ]
        print("Setting local vector {}...".format(target), flush=True)
        cb.set_global(target[0], target[1], target[2], target[3], target[4], target[5])
        input("Press enter to stop running...")
        cb.set_global(0, 0, 0, 0, 0, 0)

    except KeyboardInterrupt:
        print("")
        print("Interrupted by user")
        cb.set_global(0, 0, 0, 0, 0, 0)
        exit(1)

    exit(0)
    

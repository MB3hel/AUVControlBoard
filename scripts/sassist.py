from controlboard import ControlBoard
import traceback
import time
import controlboard


controlboard.print_all_msgs = True


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

        print("Setting mode to SASSIST...", end="", flush=True)
        if cb.set_mode(ControlBoard.Mode.SASSIST):
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

        print("Sensor Status:")
        print("  IMU:   {}".format("Connected" if cb.imu_connected else "Not Connected"))
        print("  Depth: {}".format("Connected" if cb.depth_connected else "Not Connected"))


        # PID Tunings (all gains to zero to disable)
        #                     kP      kI      kD     kF       MaxSpeed
        cb.tune_depth_hold(   0,       0,      0,      0,      1)
        cb.tune_pitch_hold(   0,       0,      0,      0,      1)
        cb.tune_roll_hold(    0,       0,      0,      0,      1)

        #           x       y       yaw     pitch_target    roll_target    depth_target        
        target = [  0,      0,      0,      0,               0,               0   ]
        print("Setting target vector {}...".format(target), flush=True)
        cb.set_sassist(target[0], target[1], target[2], target[3], target[4], target[5])
        input("Press enter to stop running...")
        cb.set_mode(ControlBoard.Mode.GLOBAL)
        cb.set_global(0, 0, 0, 0, 0, 0)

    except KeyboardInterrupt:
        print("")
        print("Interrupted by user")
        cb.set_mode(ControlBoard.Mode.GLOBAL)
        cb.set_global(0, 0, 0, 0, 0, 0)
        exit(1)

    exit(0)
    

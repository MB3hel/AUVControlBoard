from controlboard import ControlBoard
import traceback
import time
import controlboard


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

        print("Setting mode to RAW...", end="", flush=True)
        if cb.set_mode(ControlBoard.Mode.RAW):
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
    except KeyboardInterrupt:
        print("")
        print("Interrupted by user")
        exit(1)

    print("")

    # Setup and run dry run thruster tests
    try:
        # Get speed from user
        speed = input("Speed (-1.0 to 1.0; default = 0.3): ")
        try:
            speed = float(speed)
            if speed < -1.0:
                speed = -1.0
            elif speed > 1.0:
                speed = 1.0
        except:
            speed = 0.3

        # Get duration from user
        duration = input("Duration (seconds; default = 1.0): ")
        try:
            duration = float(duration)
        except:
            duration = 1.0

        # Print configuration and instructions
        print("")
        print("Speed = {}%".format(int(100 * speed)))
        print("Duration = {} sec".format(duration))
        print("")
        print("Enter a thruster (1-8) to run or q to exit.")
        print("Ctrl+C to stop thrusters and exit at any time.")
        print("")

        # Repeatedly prompt for thruster number and move that thruster
        while True:
            t = input("Thruster: ")
            if t == "q" or t == "Q":
                print("Quitting.")
                break
            try:
                t = int(t)
            except:
                print("Invalid thruster number.")
                continue
            if t < 0 or t > 8:
                print("Invalid thruster number.")
                continue
            speeds = [0] * 8
            speeds[t-1] = speed
            cb.set_raw(speeds[0], speeds[1], speeds[2], speeds[3], speeds[4], speeds[5], speeds[6], speeds[7])
            start_time = time.time()
            while time.time() - start_time < duration:
                time.sleep(0.1)
            cb.set_raw(0, 0, 0, 0, 0, 0, 0, 0)
        cb.set_raw(0, 0, 0, 0, 0, 0, 0, 0)
    except KeyboardInterrupt:
        cb.set_raw(0, 0, 0, 0, 0, 0, 0, 0)
        print("")
        print("Interrupted by user")
        exit(1)
    exit(0)
    

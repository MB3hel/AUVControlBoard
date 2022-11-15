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

        while True:
            cb.refresh_sensor_status()
            print("Sensor Status:")
            print("  IMU:   {}".format("Connected" if cb.imu_connected else "Not Connected"))
            print("  Depth: {}".format("Connected" if cb.depth_connected else "Not Connected"))
            time.sleep(1)
    except KeyboardInterrupt:
        exit(0)
    

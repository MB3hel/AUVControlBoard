import serial
import sys
import time

port = ""
if len(sys.argv) == 1:
    port = "/dev/ttyACM0"
else:
    port = sys.argv[1]

ser = serial.Serial(port, 1200)
time.sleep(0.1)
ser.close()

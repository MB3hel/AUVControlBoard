import serial

opened = False
try:
    ser: serial.Serial = serial.Serial("/dev/ttyACM0")
    opened = True
    print("Resetting")
    ser.write(b'\xfdRESET}\xb2\xfe')
    ser.close()
except:
    if not opened:
        print("Error connecting to control board!")

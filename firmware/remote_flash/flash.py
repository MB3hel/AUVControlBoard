#!/usr/bin/env python3

from serial import Serial
import time
import argparse
import os


def reset_to_bootloader(port):
    ser = Serial(port, 1200)
    time.sleep(0.5)
    ser.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port", type=str)
    parser.add_argument("firmware", type=str)
    args = parser.parse_args()

    script_dir = os.path.dirname(__file__)
    reset_to_bootloader(args.port)
    time.sleep(2)
    os.system("python3 {}/uf2conv.py -b 0x4000 {}".format(script_dir, args.firmware))


if __name__ == "__main__":
    main()
    
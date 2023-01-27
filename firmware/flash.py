#!/usr/bin/env python3

import subprocess
import os
import argparse
import sys
import time


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Flash control board firmware.")
    parser.add_argument("cboard_rev", type=str.lower, choices=["v1", "v2"], help="Which control board revision to flash firmware for.")
    parser.add_argument("config", type=str.lower, choices=["debug", "release", "minsizerel", "relwithdebinfo"], help="Which build type to flash.")
    parser.add_argument("-u", dest="uploader", metavar="uploader", default="auto", type=str, choices=["auto", "bossa", "uf2conv", "dfu-util", "stm32-dfu", "stm32-stlink2"], help="Which tool to use to upload firmware. Available tools depend on target board.")
    parser.add_argument("-p", dest="port", metavar="port", type=str, default="auto", help="Port to upload to. Meaning depends on uploader.")
    args = parser.parse_args()

    script_dir = os.path.dirname(__file__)
    
    # Fix config case to match cmake folder names (needed for case sensitive filesystems)
    if args.config.lower() == "debug":
        args.config = "Debug"
    elif args.config.lower() == "release":
        args.config = "Release"
    elif args.config == "minsizerel":
        args.config = "MinSizeRel"
    elif args.config == "relwithdebinfo":
        args.config = "RelWithDebInfo"

    if args.cboard_rev == "v1":
        # Assign default uploader for v1
        if args.uploader == "auto":
            args.uploader = "bossa"

        # Flash using specified uploader
        if args.uploader == "bossa":
            # bossac -o 0x4000 -e -w -v -b -R firmware.bin
            # If port is not auto, port is passed with -p flag
            cmd = ["bossac", "-o", "0x4000", "-e", "-w", "-v", "-b", "-R"]
            if args.port != "auto":
                cmd.extend(["-p", args.port])
            cmd.append(os.path.join(script_dir, "build", "v1", args.config, "ControlBoard.bin"))
            print(" ".join(cmd))
            subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr, stdin=sys.stdin, shell=False)
        elif args.uploader == "uf2conv":
            # uf2conv firmware.hex
            # The port argument has no effect
            cmd = [sys.executable, os.path.join(script_dir, "tools", "uf2conv", "uf2conv.py")]
            cmd.append(os.path.join(script_dir, "build", "v1", args.config, "ControlBoard.hex"))
            print(" ".join(cmd))
            subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr, stdin=sys.stdin, shell=False)
        else:
            print("ERROR: Unsupported uploader for the given revision!")
            exit(1)
    elif args.cboard_rev == "v2":
        # Assign default uploader for v2
        if args.uploader == "auto":
            args.uploader = "dfu-util"
        
        # Flash using specified uploader
        if args.uploader == "dfu-util":
            # dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D ControlBoard.bin
            # Replace -a 0 with -a args.port if args.port is not auto
            cmd = ["dfu-util", "-d", "0483:df11"]
            cmd.extend(["-a", args.port if args.port != "auto" else "0"])
            cmd.extend(["-s", "0x08000000:leave"])
            cmd.append("-D")
            cmd.append(os.path.join(script_dir, "build", "v2", args.config, "ControlBoard.bin"))
            print(" ".join(cmd))
            subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr, stdin=sys.stdin, shell=False)
        elif args.uploader == "stm32-dfu":
            # STM32_Programmer_CLI -c port=USB1 -e -w ControlBoard.hex -v -s
            # In port=USB[n] change [n] to args.port if args.port is not auto
            cmd = ["STM32_Programmer_CLI", "-c"]
            cmd.append("port=USB{}".format(args.port if args.port != "auto" else "1"))
            cmd.extend(["-e", "-w"])
            cmd.append(os.path.join(script_dir, "build", "v2", args.config, "ControlBoard.hex"))
            cmd.extend(["-v", "-s"])
            print(" ".join(cmd))
            subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr, stdin=sys.stdin, shell=False)
        elif args.uploader == "stm32-stlink2":
            # STM32_Programmer_CLI -c port=SWD freq=4000 index=0 -e -w ControlBoard.hex -v -s
            # Change index if args.port is not auto
            cmd = ["STM32_Programmer_CLI", "-c", "port=SWD", "freq=4000"]
            cmd.append("index={}".format(args.port if args.port != "auto" else "0"))
            cmd.extend(["-e", "-w"])
            cmd.append(os.path.join(script_dir, "build", "v2", args.config, "ControlBoard.hex"))
            cmd.extend(["-v", "-s"])
            print(" ".join(cmd))
            subprocess.run(cmd, stdout=sys.stdout, stderr=sys.stderr, stdin=sys.stdin, shell=False)
        else:
            print("ERROR: Unsupported uploader for the given revision!")
            exit(1)

    exit(0)
            

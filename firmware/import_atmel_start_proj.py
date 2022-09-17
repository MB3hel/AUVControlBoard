#!/usr/bin/env python3

from genericpath import isdir
import os
import shutil
import glob
import sys
import tempfile


def copyglob(glb: str, dest: str):
    if not os.path.exists(dest):
        os.makedirs(dest)
    for item in glob.glob(glb, recursive=True):
        if os.path.isdir(item):
            shutil.copytree(item, "{}/{}".format(dest, os.path.basename(item)))
        else:
            shutil.copy2(item, dest)


def copyreplacefile(src: str, dest: str):
    if os.path.exists(dest):
        os.remove(dest)
    if not os.path.exists(os.path.dirname(dest)):
        os.makedirs(os.path.dirname(dest))
    shutil.copy2(src, dest)


if __name__ == "__main__":
    script_dir = os.path.dirname(__file__).replace("\\", "/")

    if len(sys.argv) != 2:
        print("Requires exactly one argument: Path to downloaded project.")
        exit(1)

    if not os.path.exists(sys.argv[1]):
        print("File does not exist.")
        exit(1)

    with tempfile.TemporaryDirectory() as tempdirname:
        # Extract zip
        shutil.unpack_archive(sys.argv[1], tempdirname, format="zip")

        # Delete old libraries
        shutil.rmtree("{}/include/cmsis".format(script_dir), ignore_errors=True)
        shutil.rmtree("{}/include/samd51a".format(script_dir), ignore_errors=True)
        shutil.rmtree("{}/src/samd51a".format(script_dir), ignore_errors=True)
        shutil.rmtree("{}/include/asf4".format(script_dir), ignore_errors=True)
        shutil.rmtree("{}/src/asf4".format(script_dir), ignore_errors=True)
        shutil.rmtree("{}/include/ast".format(script_dir), ignore_errors=True)
        shutil.rmtree("{}/src/ast".format(script_dir), ignore_errors=True)

        # Copy CMSIS library
        print("Copying CMSIS...")
        copyglob("{}/CMSIS/Core/Include/*.h".format(tempdirname), "{}/include/cmsis/".format(script_dir))
        
        # Copy samd51a DFP
        print("Copying SAMD51A_DFP...")
        copyglob("{}/samd51a/include/*".format(tempdirname), "{}/include/samd51a/".format(script_dir))
        copyglob("{}/samd51a/gcc/*.c".format(tempdirname), "{}/src/samd51a".format(script_dir))
        copyglob("{}/samd51a/gcc/gcc/*.c".format(tempdirname), "{}/src/samd51a".format(script_dir))

        # Copy ASF4 hal Libraries
        print("Copying ASF4 hal...")
        copyglob("{}/hal/include/*.h".format(tempdirname), "{}/include/asf4/hal".format(script_dir))
        copyglob("{}/hal/src/*.c".format(tempdirname), "{}/src/asf4/hal".format(script_dir))
        copyglob("{}/hal/utils/include/*.h".format(tempdirname), "{}/include/asf4/hal/utils".format(script_dir))
        copyglob("{}/hal/utils/src/*.c".format(tempdirname), "{}/src/asf4/hal/utils".format(script_dir))

        # Copy ASF4 hpl Libraries
        print("Copying ASF4 hpl...")
        copyglob("{}/hpl/**/*.h".format(tempdirname), "{}/include/asf4/hpl".format(script_dir))
        copyglob("{}/hpl/**/*.c".format(tempdirname), "{}/src/asf4/hpl".format(script_dir))

        # Copy ASF4 hri
        print("Copying ASF4 hri...")
        copyglob("{}/hri/*.h".format(tempdirname), "{}/include/asf4/hri".format(script_dir))


        # Copy ASF4 config
        print("Copying ASF4 config...")
        copyglob("{}/config/*.h".format(tempdirname), "{}/include/asf4/config".format(script_dir))

        # Copy src files
        print("Copying ASF4 init files...")
        copyreplacefile("{}/atmel_start.h".format(tempdirname), "{}/include/ats/atmel_start.h".format(script_dir))
        copyreplacefile("{}/atmel_start_pins.h".format(tempdirname), "{}/include/ats/atmel_start_pins.h".format(script_dir))
        copyreplacefile("{}/driver_init.h".format(tempdirname), "{}/include/ats/driver_init.h".format(script_dir))
        copyreplacefile("{}/atmel_start.c".format(tempdirname), "{}/src/ats/atmel_start.c".format(script_dir))
        copyreplacefile("{}/driver_init.c".format(tempdirname), "{}/src/ats/driver_init.c".format(script_dir))

        # Copy usb files
        if os.path.exists("{}/usb".format(tempdirname)):
            print("Copying ASF4 usb files...")
            copyglob("{}/usb/**/*.h".format(tempdirname), "{}/include/asf4/usb".format(script_dir))
            copyglob("{}/usb/**/*.c".format(tempdirname), "{}/src/asf4/usb".format(script_dir))
            copyreplacefile("{}/usb_start.c".format(tempdirname), "{}/src/ats/usb_start.c".format(script_dir))
            copyreplacefile("{}/usb_start.h".format(tempdirname), "{}/include/ats/usb_start.h".format(script_dir))

    exit(0)

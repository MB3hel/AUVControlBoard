#!/usr/bin/env python3

from genericpath import isdir
import os
import shutil
import glob
import sys
import tempfile


def copyglob(glb: str, dest: str):
    if not os.path.exists(dest):
        os.mkdir(dest)
    for item in glob.glob(glb, recursive=True):
        if os.path.isdir(item):
            shutil.copytree(item, "{}/{}".format(dest, os.path.basename(item)))
        else:
            shutil.copy2(item, dest)


def copyreplacefile(src: str, dest: str):
    if os.path.exists(dest):
        os.remove(dest)
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
        shutil.rmtree("{}/lib/cmsis/include".format(script_dir), ignore_errors=True)
        shutil.rmtree("{}/lib/samd51a/include".format(script_dir), ignore_errors=True)
        shutil.rmtree("{}/lib/samd51a/src".format(script_dir), ignore_errors=True)
        shutil.rmtree("{}/lib/asf4/include".format(script_dir), ignore_errors=True)
        shutil.rmtree("{}/lib/asf4/src".format(script_dir), ignore_errors=True)

        # Copy CMSIS library
        print("Copying CMSIS...")
        copyglob("{}/CMSIS/Core/Include/*.h".format(tempdirname), "{}/lib/cmsis/include".format(script_dir))
        
        # Copy samd51a DFP
        print("Copying SAMD51A_DFP...")
        copyglob("{}/samd51a/include/*".format(tempdirname), "{}/lib/samd51a/include".format(script_dir))
        copyglob("{}/samd51a/gcc/*.c".format(tempdirname), "{}/lib/samd51a/src".format(script_dir))
        copyglob("{}/samd51a/gcc/gcc/*".format(tempdirname), "{}/lib/samd51a/src".format(script_dir))

        # Copy ASF4 hal Libraries
        print("Copying ASF4 hal...")
        copyglob("{}/hal/include/*.h".format(tempdirname), "{}/lib/asf4/include".format(script_dir))
        copyglob("{}/hal/src/*.c".format(tempdirname), "{}/lib/asf4/src".format(script_dir))
        copyglob("{}/hal/utils/include/*.h".format(tempdirname), "{}/lib/asf4/include".format(script_dir))
        copyglob("{}/hal/utils/src/*.c".format(tempdirname), "{}/lib/asf4/src".format(script_dir))

        # Copy ASF4 hpl Libraries
        print("Copying ASF4 hpl...")
        copyglob("{}/hpl/**/*.h".format(tempdirname), "{}/lib/asf4/include".format(script_dir))
        copyglob("{}/hpl/**/*.c".format(tempdirname), "{}/lib/asf4/src".format(script_dir))

        # Copy ASF4 hri
        print("Copying ASF4 hri...")
        copyglob("{}/hri/*.h".format(tempdirname), "{}/lib/asf4/include".format(script_dir))


        # Copy ASF4 config
        print("Copying ASF4 config...")
        copyglob("{}/config/*.h".format(tempdirname), "{}/lib/asf4/include".format(script_dir))

        # Copy src files
        print("Copying ASF4 init files...")
        copyreplacefile("{}/atmel_start.h".format(tempdirname), "{}/include/atmel_start.h".format(script_dir))
        copyreplacefile("{}/atmel_start_pins.h".format(tempdirname), "{}/include/atmel_start_pins.h".format(script_dir))
        copyreplacefile("{}/driver_init.h".format(tempdirname), "{}/include/driver_init.h".format(script_dir))
        copyreplacefile("{}/atmel_start.c".format(tempdirname), "{}/src/atmel_start.c".format(script_dir))
        copyreplacefile("{}/driver_init.c".format(tempdirname), "{}/src/driver_init.c".format(script_dir))

    exit(0)

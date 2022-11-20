#!/usr/bin/env python3

import os
import shutil
import glob
import sys
import traceback
from io import StringIO


def replace_in_file(file, old: str, new: str):
    contents = ""
    with open(file, 'r') as f:
        contents = f.read()
    contents = contents.replace(old, new)
    with open(file, 'w') as f:
        f.write(contents)

def copyglob(glb: str, dest: str):
    glob_res = glob.glob(glb, recursive=True)
    if len(glob_res) == 0:
        return
    if not os.path.exists(dest):
        os.makedirs(dest)
    for item in glob_res:
        if os.path.isdir(item):
            shutil.copytree(item, "{}/{}".format(dest, os.path.basename(item)))
        else:
            shutil.copy2(item, dest)


def update_controlboard_v1():
    # Note: Following this gude for building harmony v3 project with gcc
    # https://microchip-mplab-harmony.github.io/quick_docs/source/migration/build_harmony_3_project_with_gcc/readme.html
    script_dir = os.path.dirname(__file__)
    generator_proj = os.path.join(script_dir, "generator_projects", "ControlBoard_v1")
    if not os.path.exists(os.path.join(generator_proj, "firmware", "src", "main.c")):
        print("Project not generated. Use MCC Standalone to generate code first.")
        return 1


    ####################################################################################################################
    # Remove old generated stuff
    ####################################################################################################################
    src_dest = os.path.join(script_dir, "src", "v1_generated")
    inc_dest = os.path.join(script_dir, "include", "v1_generated")
    linker_dest = os.path.join(script_dir, "linker", "ControlBoard_v1.ld")
    if os.path.exists(src_dest):
        shutil.rmtree(src_dest)
    if os.path.exists(inc_dest):
        shutil.rmtree(inc_dest)
    if os.path.exists(linker_dest):
        os.remove(linker_dest)
    os.mkdir(src_dest)
    os.mkdir(inc_dest)

    ####################################################################################################################
    # Copy files to appropriate directories
    ####################################################################################################################
    # Linker script
    shutil.copy(os.path.join(generator_proj, "supplemental", "samd51g19a_flash.ld"), linker_dest)

    # Modify linker script because first 16k reserved for UF2 bootloader
    replace_in_file(linker_dest, "rom      (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00080000", "rom      (rx)  : ORIGIN = 0x00004000, LENGTH = 0x0007C000")

    # gcc startup and system sources
    shutil.copy(os.path.join(generator_proj, "supplemental", "startup_samd51g19a.c"), src_dest)
    shutil.copy(os.path.join(generator_proj, "supplemental", "system_samd51g19a.c"), src_dest)

    # Patch system file so SystemCoreClock is right based on configured project
    replace_in_file(os.path.join(src_dest, "system_samd51g19a.c"), "#define __SYSTEM_CLOCK    (48000000)", "")
    replace_in_file(os.path.join(src_dest, "system_samd51g19a.c"), "__SYSTEM_CLOCK", "CPU_CLOCK_FREQUENCY")
    replace_in_file(os.path.join(src_dest, "system_samd51g19a.c"), '#include "samd51g19a.h"', '#include "samd51g19a.h"\n#include "definitions.h"')

    # CMSIS Core
    shutil.copytree(os.path.join(generator_proj, "firmware", "src", "packs", "CMSIS", "CMSIS", "Core", "Include"), os.path.join(inc_dest, "CMSIS", "Core"))
    
    # Device Framework / Support Pack (DFP / DSP)
    shutil.copytree(os.path.join(generator_proj, "firmware", "src", "packs", "ATSAMD51G19A_DFP"), os.path.join(inc_dest, "ATSAMD51G19A_DFP"))

    # Loose files in config/default
    harmony_src_dest = os.path.join(src_dest, "mcc_harmony")
    harmony_inc_dest = os.path.join(inc_dest, "mcc_harmony")
    os.mkdir(harmony_src_dest)
    os.mkdir(harmony_inc_dest)
    shutil.copy(os.path.join(generator_proj, "firmware", "src", "config", "default", "definitions.h"), harmony_inc_dest)
    shutil.copy(os.path.join(generator_proj, "firmware", "src", "config", "default", "device.h"), harmony_inc_dest)
    shutil.copy(os.path.join(generator_proj, "firmware", "src", "config", "default", "device_cache.h"), harmony_inc_dest)
    shutil.copy(os.path.join(generator_proj, "firmware", "src", "config", "default", "device_vectors.h"), harmony_inc_dest)
    shutil.copy(os.path.join(generator_proj, "firmware", "src", "config", "default", "initialization.c"), harmony_src_dest)
    shutil.copy(os.path.join(generator_proj, "firmware", "src", "config", "default", "libc_syscalls.c"), harmony_src_dest)
    shutil.copy(os.path.join(generator_proj, "firmware", "src", "config", "default", "toolchain_specifics.h"), harmony_inc_dest)

    # Comment pragma configs in initialization.c (not supported by gcc)
    replace_in_file(os.path.join(harmony_src_dest, "initialization.c"), "#pragma config", "// #pragma config")

    # config/default/peripheral
    os.mkdir(os.path.join(harmony_src_dest, "peripheral"))
    os.mkdir(os.path.join(harmony_inc_dest, "peripheral"))
    for name in os.listdir(os.path.join(generator_proj, "firmware", "src", "config", "default", "peripheral")):
        if os.path.isdir(os.path.join(generator_proj, "firmware", "src", "config", "default", "peripheral", name)):
            srcglb = "{}/*.c".format(os.path.join(generator_proj, "firmware", "src", "config", "default", "peripheral", name).replace("\\", "/"))
            incglb = "{}/*.h".format(os.path.join(generator_proj, "firmware", "src", "config", "default", "peripheral", name).replace("\\", "/"))
            copyglob(srcglb, os.path.join(harmony_src_dest, "peripheral"))
            copyglob(incglb, os.path.join(harmony_inc_dest, "peripheral"))

    for name in os.listdir(os.path.join(generator_proj, "firmware", "src", "config", "default", "peripheral")):

        # Patching include paths in definitions.h
        replace_in_file(os.path.join(inc_dest, "mcc_harmony", "definitions.h"), '#include "peripheral/{}/'.format(name), '#include "')

        # Patching include paths in all copied .c files
        for file in glob.glob("{}/mcc_harmony/peripheral/*.c".format(src_dest.replace("\\", "/")), recursive=True):
            replace_in_file(file, '#include "peripheral/{}/'.format(name), '#include "')

    # Patch libc_syscalls.c for gcc
    contents = ""
    with open(os.path.join(harmony_src_dest, "libc_syscalls.c"), 'r') as f:
        contents = f.read()
    idx = contents.rfind("#ifdef __cplusplus")
    idx = contents.rfind("\n", 0, idx)
    patch = "extern int _end;\nextern caddr_t _sbrk(int incr);\ncaddr_t _sbrk(int incr){\n    static unsigned char *heap = NULL;\n    unsigned char *       prev_heap;\n    if (heap == NULL) {\n	    heap = (unsigned char *)&_end;\n    }\n    prev_heap = heap;\n    heap += incr;\n    return (caddr_t)prev_heap;\n}"
    contents = contents[:idx-1] + patch + contents[idx:]
    with open(os.path.join(harmony_src_dest, "libc_syscalls.c"), 'w') as f:
        f.write(contents)

    # Create empty versions of files that are not copied (so include errors don't happen)
    with open(os.path.join(inc_dest, "mcc_harmony", "interrupts.h"), 'w') as f:
        f.write("#pragma once\n\n//Empty")


def update_controlboard_v2():
    script_dir = os.path.dirname(__file__)
    generator_proj = os.path.join(script_dir, "generator_projects", "ControlBoard_v2")
    if not os.path.exists(os.path.join(generator_proj, "Makefile")):
        print("Project not generated. Run STM32CubeMX and generate code first.")
        return 1
    
    ####################################################################################################################
    # Remove old generated stuff
    ####################################################################################################################
    src_dest = os.path.join(script_dir, "src", "v2_generated")
    inc_dest = os.path.join(script_dir, "include", "v2_generated")
    linker_dest = os.path.join(script_dir, "linker", "ControlBoard_v2.ld")
    if os.path.exists(src_dest):
        shutil.rmtree(src_dest)
    if os.path.exists(inc_dest):
        shutil.rmtree(inc_dest)
    if os.path.exists(linker_dest):
        os.remove(linker_dest)
    os.mkdir(src_dest)
    os.mkdir(inc_dest)

    ####################################################################################################################
    # Copy files to appropriate directories
    ####################################################################################################################
    # Linker script
    shutil.copy(os.path.join(generator_proj, "STM32F411CEUx_FLASH.ld"), linker_dest)

    # Startup script
    shutil.copy(os.path.join(generator_proj, "startup_stm32f411xe.s"), os.path.join(src_dest))

    # Core
    shutil.copytree(os.path.join(generator_proj, "Core", "Inc"), os.path.join(inc_dest, "Core"))
    shutil.copytree(os.path.join(generator_proj, "Core", "Src"), os.path.join(src_dest, "Core"))
   
    # Remove interrupt handlers
    os.remove(os.path.join(inc_dest, "Core", "stm32f4xx_it.h"))
    os.remove(os.path.join(src_dest, "Core", "stm32f4xx_it.c"))

    # Rename main.h and main.c
    os.rename(os.path.join(inc_dest, "Core", "main.h"), os.path.join(inc_dest, "Core", "stm32cubemx_main.h"))
    os.rename(os.path.join(src_dest, "Core", "main.c"), os.path.join(src_dest, "Core", "stm32cubemx_main.c"))
    
    # Fix main.h includes
    for file in os.listdir(os.path.join(src_dest, "Core")):
        if file.endswith(".c"):
            replace_in_file(os.path.join(src_dest, "Core", file), "#include \"main.h\"", "#include \"stm32cubemx_main.h\"")

    # Change name of main function
    replace_in_file(os.path.join(src_dest, "Core", "stm32cubemx_main.c"), "int main(void)", "void stm32cubemx_main(void)")
    
    # Remove infinite while loop from stm32cubemx_main
    content = ""
    with open(os.path.join(src_dest, "Core", "stm32cubemx_main.c"), 'r') as f:
        content = f.read()
    main_idx = content.find("void stm32cubemx_main(void)")
    while_idx = content.find("while (1)", main_idx)
    while_end_idx = content.find("}", while_idx)
    content = content[:while_idx] + content[while_end_idx + 1:]
    with open(os.path.join(src_dest, "Core", "stm32cubemx_main.c"), 'w') as f:
        f.write(content)

    # Drivers/CMSIS
    shutil.copytree(os.path.join(generator_proj, "Drivers", "CMSIS", "Include"), os.path.join(inc_dest, "Drivers", "CMSIS"))
    
    # Drivers/CMSIS/Device/ST/STM32F4xx
    shutil.copytree(os.path.join(generator_proj, "Drivers", "CMSIS", "Device", "ST", "STM32F4xx", "Include"), os.path.join(inc_dest, "Drivers", "CMSIS", "Device", "ST", "STM32F4xx"))
    shutil.copytree(os.path.join(generator_proj, "Drivers", "CMSIS", "Device", "ST", "STM32F4xx", "Source"), os.path.join(src_dest, "Drivers", "CMSIS", "Device", "ST", "STM32F4xx"))
    
    # Drivers/STM32F4xx_HAL_Driver
    shutil.copytree(os.path.join(generator_proj, "Drivers", "STM32F4xx_HAL_Driver", "Inc"), os.path.join(inc_dest, "Drivers", "STM32F4xx_HAL_Driver"))
    shutil.copytree(os.path.join(generator_proj, "Drivers", "STM32F4xx_HAL_Driver", "Src"), os.path.join(src_dest, "Drivers", "STM32F4xx_HAL_Driver"))

    return 0

def main():    
    print("Update Generator Project:")
    print("    A. ControlBoard v1 (Adafruit ItsyBitsy M4)")
    print("    B. ControlBoard v2 (WeAct Studio Black Pill STM32F411)")
    
    cont = True
    while cont:
        res = input("Select Project To Update: ").upper()
        cont = False
        if res == "A":
            return update_controlboard_v1()
        elif res == "B":
            return update_controlboard_v2()
        else:
            cont = True
    return 0
            


if __name__ == "__main__":
    ret = 0
    try:
        ret = main()
    except:
        traceback.print_exc()
        sys.exit(2)
    sys.exit(ret)

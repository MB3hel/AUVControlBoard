#!/usr/bin/env python3

import os
import shutil
import glob
import sys
import traceback
from io import StringIO


################################################################################
# Helper functions
################################################################################

# Replace any instance of old with new in a given file
def replace_in_file(file, old: str, new: str):
    contents = ""
    with open(file, 'r') as f:
        contents = f.read()
    contents = contents.replace(old, new)
    with open(file, 'w') as f:
        f.write(contents)

# Replaces lines starting with a certain prefix with a new line in a given file
def replace_prefix_in_file(file, line_prefix: str, new_line: str):
    contents = StringIO()
    with open(file, 'r') as f:
        line = f.readline()
        while line != "" and line is not None:
            if line.startswith(line_prefix):
                contents.write(new_line)
                contents.write("\n")
            else:
                contents.write(line)
            line = f.readline()
    contents.seek(0)
    with open(file, 'w') as f:
        f.write(contents.read())

# Copy a set of files matching a (recursive) glob to the given destination directory
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


################################################################################
# Control Board v1 (MCC import)
################################################################################

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
    dest = os.path.join(script_dir, "thirdparty", "v1_generated")
    if os.path.exists(dest):
        shutil.rmtree(dest)
    os.mkdir(dest)

    ####################################################################################################################
    # Copy files to appropriate directories
    ####################################################################################################################
    # Linker script
    shutil.copy(os.path.join(generator_proj, "supplemental", "samd51g19a_flash.ld"), dest)

    # Modify linker script because first 16k reserved for UF2 bootloader
    replace_in_file(os.path.join(dest, "samd51g19a_flash.ld"), "rom      (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00080000", "rom      (rx)  : ORIGIN = 0x00004000, LENGTH = 0x0007C000")

    # Set explicit stack size in linker script
    replace_prefix_in_file(os.path.join(dest, "samd51g19a_flash.ld"), "STACK_SIZE = ", "STACK_SIZE = 0xC000;")

    # Add __bss_start and __bss_end__ definitions to linker script
    replace_in_file(os.path.join(dest, "samd51g19a_flash.ld"), "_szero = .;", "_szero = .;\n        __bss_start__ = .;")
    replace_in_file(os.path.join(dest, "samd51g19a_flash.ld"), "_ezero = .;", "_ezero = .;\n        __bss_end__ = .;")


    # gcc startup and system sources
    shutil.copy(os.path.join(generator_proj, "supplemental", "startup_samd51g19a.c"), dest)
    shutil.copy(os.path.join(generator_proj, "supplemental", "system_samd51g19a.c"), dest)

    # SVD file for debugging
    shutil.copy(os.path.join(generator_proj, "supplemental", "ATSAMD51G19A.svd"), dest)

    # MCC generated code
    shutil.copytree(os.path.join(generator_proj, "firmware", "src", "config", "default"), os.path.join(dest, "mcc"))
    shutil.rmtree(os.path.join(dest, "mcc", "default.mhc"))
    shutil.rmtree(os.path.join(dest, "mcc", "stdio"))
    os.remove(os.path.join(dest, "mcc", "ATSAMD51G19A.ld"))
    os.remove(os.path.join(dest, "mcc", "libc_syscalls.c"))
    shutil.copy(os.path.join(generator_proj, "supplemental", "libc_syscalls.c"), os.path.join(dest, "mcc"))
    os.remove(os.path.join(dest, "mcc", "device_vectors.h"))
    os.remove(os.path.join(dest, "mcc", "exceptions.c"))
    os.remove(os.path.join(dest, "mcc", "interrupts.c"))
    os.remove(os.path.join(dest, "mcc", "startup_xc32.c"))
    os.remove(os.path.join(dest, "mcc", "harmony-manifest-success.yml"))
    os.remove(os.path.join(dest, "mcc", "pin_configurations.csv"))

    # Remove unknown pragmas from initialization.c
    replace_in_file(os.path.join(dest, "mcc", "initialization.c"), "#pragma config", "// #pragma config")

    # CMSIS pack
    shutil.copytree(os.path.join(generator_proj, "firmware", "src", "packs", "CMSIS"), os.path.join(dest, "packs", "CMSIS"))
    shutil.copytree(os.path.join(generator_proj, "firmware", "src", "packs", "ATSAMD51G19A_DFP"), os.path.join(dest, "packs", "ATSAMD51G19A_DFP"))

    return 0


################################################################################
# Control Board v2 (STM32CubeMX import)
################################################################################

def update_controlboard_v2():
    script_dir = os.path.dirname(__file__)
    generator_proj = os.path.join(script_dir, "generator_projects", "ControlBoard_v2")
    if not os.path.exists(os.path.join(generator_proj, "Makefile")):
        print("Project not generated. Run STM32CubeMX and generate code first.")
        return 1
    
    ####################################################################################################################
    # Remove old generated stuff
    ####################################################################################################################
    dest = os.path.join(script_dir, "thirdparty", "v2_generated")
    if os.path.exists(dest):
        shutil.rmtree(dest)
    os.mkdir(dest)

    ####################################################################################################################
    # Copy files to appropriate directories
    ####################################################################################################################
    # Linker script
    # DO NOT COPY THE GENERATED ONE. USING MODIFIED ONE IN SUPPLEMENTAL (EEPROM EMULATION SUPPORT)
    # shutil.copy(os.path.join(generator_proj, "STM32F411CEUx_FLASH.ld"), dest)

    # Startup script
    shutil.copy(os.path.join(generator_proj, "startup_stm32f411xe.s"), dest)

    # Core tree
    shutil.copytree(os.path.join(generator_proj, "Core"), os.path.join(dest, "Core"))

    # Remove interrupt handlers
    os.remove(os.path.join(dest, "Core", "Inc", "stm32f4xx_it.h"))
    os.remove(os.path.join(dest, "Core", "Src", "stm32f4xx_it.c"))

    # Drivers tree
    shutil.copytree(os.path.join(generator_proj, "Drivers"), os.path.join(dest, "Drivers"))

    # Supplemental files
    shutil.copy(os.path.join(generator_proj, "supplemental", "libc_syscalls.c"), dest)
    shutil.copy(os.path.join(generator_proj, "supplemental", "STM32F411.svd"), dest)
    shutil.copy(os.path.join(generator_proj, "supplemental", "STM32F411CEUx_FLASH.ld"), dest)
    shutil.copytree(os.path.join(generator_proj, "supplemental", "emueeprom"), os.path.join(dest, "emueeprom"))


    # --------------------------------------------------------------------------
    # Modify main into stm32cubemx_main
    # --------------------------------------------------------------------------

    # Rename main.h and main.c
    os.rename(os.path.join(dest, "Core", "Inc", "main.h"), os.path.join(dest, "Core", "Inc", "stm32cubemx_main.h"))
    os.rename(os.path.join(dest, "Core", "Src", "main.c"), os.path.join(dest, "Core", "Src", "stm32cubemx_main.c"))

    # Fix any file including main.h
    for file in os.listdir(os.path.join(dest, "Core", "Src")):
        if file.endswith(".c"):
            replace_in_file(os.path.join(dest, "Core", "Src", file), "#include \"main.h\"", "#include \"stm32cubemx_main.h\"")

    # Disable watchdog startup on system boot (should be handled in v2/wdt.c)
    replace_in_file(os.path.join(dest, "Core", "Src", "stm32cubemx_main.c"), "MX_IWDG_Init();", "// MX_IWDG_Init();")
    replace_in_file(os.path.join(dest, "Core", "Src", "stm32cubemx_main.c"), "static void MX_IWDG_Init(void)", "void MX_IWDG_Init(void)")

    # Change name of main function
    replace_in_file(os.path.join(dest, "Core", "Src", "stm32cubemx_main.c"), "int main(void)", "void stm32cubemx_main(void)")

    # Remove infinite while loop from stm32cubemx_main
    content = ""
    with open(os.path.join(dest, "Core", "Src", "stm32cubemx_main.c"), 'r') as f:
        content = f.read()
    main_idx = content.find("void stm32cubemx_main(void)")
    while_idx = content.find("while (1)", main_idx)
    while_end_idx = content.find("}", while_idx)
    content = content[:while_idx] + content[while_end_idx + 1:]
    with open(os.path.join(dest, "Core", "Src", "stm32cubemx_main.c"), 'w') as f:
        f.write(content)

    # --------------------------------------------------------------------------

    return 0


################################################################################
# Entry point
################################################################################

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

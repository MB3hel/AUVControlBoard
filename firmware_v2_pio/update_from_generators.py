import os
import shutil
import glob
import sys


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
    os.mkdir(os.path.join(inc_dest, "Core"))
    shutil.copy(os.path.join(generator_proj, "Core", "Inc", "stm32f4xx_hal_conf.h"), os.path.join(inc_dest, "Core"))
    os.mkdir(os.path.join(src_dest, "Core"))
    shutil.copy(os.path.join(generator_proj, "Core", "Src", "system_stm32f4xx.c"), os.path.join(src_dest, "Core"))

    # Drivers/CMSIS
    shutil.copytree(os.path.join(generator_proj, "Drivers", "CMSIS", "Include"), os.path.join(inc_dest, "Drivers", "CMSIS"))
    
    # Drivers/CMSIS/Device/ST/STM32F4xx
    shutil.copytree(os.path.join(generator_proj, "Drivers", "CMSIS", "Device", "ST", "STM32F4xx", "Include"), os.path.join(inc_dest, "Drivers", "CMSIS", "Device", "ST", "STM32F4xx"))
    shutil.copytree(os.path.join(generator_proj, "Drivers", "CMSIS", "Device", "ST", "STM32F4xx", "Source"), os.path.join(src_dest, "Drivers", "CMSIS", "Device", "ST", "STM32F4xx"))
    
    # Drivers/STM32F4xx_HAL_Driver
    shutil.copytree(os.path.join(generator_proj, "Drivers", "STM32F4xx_HAL_Driver", "Inc"), os.path.join(inc_dest, "Drivers", "STM32F4xx_HAL_Driver"))
    shutil.copytree(os.path.join(generator_proj, "Drivers", "STM32F4xx_HAL_Driver", "Src"), os.path.join(src_dest, "Drivers", "STM32F4xx_HAL_Driver"))

    print("***Make sure to manually update the init.c file from Core/Src/main.c!***")

    return 0

def main():    
    print("Update Generator Project:")
    print("    A. ControlBoard v1 (Adafruit ItsyBitsy M4)")
    print("    B. ControlBoard v2 (Black Pill STM32F411)")
    
    cont = True
    while cont:
        res = input("Select Project To Update: ").upper()
        cont = False
        if res == "A":
            print("Not Yet Implemented!")
        elif res == "B":
            return update_controlboard_v2()
        else:
            cont = True
    return 0
            


if __name__ == "__main__":
    sys.exit(main())

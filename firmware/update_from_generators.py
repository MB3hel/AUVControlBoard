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
            print("Not Yet Implemented!")
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
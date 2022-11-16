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
    script_dir = os.path.dirname(__file__)
    generator_proj = os.path.join(script_dir, "generator_projects", "ControlBoard_v1")
    if not os.path.exists(os.path.join(generator_proj, "ControlBoard_v1.atzip")):
        print("Project not generated. Use Atmel Start to generate code first.")
        return 1

    ####################################################################################################################
    # Extract the atzip file
    ####################################################################################################################
    generator_dest = os.path.join(generator_proj, "extracted")
    if os.path.exists(generator_dest):
        shutil.rmtree(generator_dest)
    os.mkdir(generator_dest)
    shutil.unpack_archive(os.path.join(generator_proj, "ControlBoard_v1.atzip"), generator_dest, "zip")
    generator_proj = generator_dest


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
    shutil.copy(os.path.join(generator_proj, "samd51a", "gcc", "gcc", "samd51g19a_flash.ld"), linker_dest)

    # Modify linker script because first 16k reserved for UF2 bootloader
    replace_in_file(linker_dest, "rom      (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00080000", "rom      (rx)  : ORIGIN = 0x00004000, LENGTH = 0x0007C000")

    # samd51a
    shutil.copytree(os.path.join(generator_proj, "samd51a", "include"), os.path.join(inc_dest, "samd51a"))
    os.mkdir(os.path.join(src_dest, "samd51a"))
    shutil.copy(os.path.join(generator_proj, "samd51a", "gcc", "system_samd51.c"), os.path.join(src_dest, "samd51a"))
    shutil.copy(os.path.join(generator_proj, "samd51a", "gcc", "gcc", "startup_samd51.c"), os.path.join(src_dest, "samd51a"))

    # CMSIS
    shutil.copytree(os.path.join(generator_proj, "CMSIS", "Core", "Include"), os.path.join(inc_dest, "CMSIS", "Core"))

    # config
    shutil.copytree(os.path.join(generator_proj, "config"), os.path.join(inc_dest, "config"))

    # hal
    shutil.copytree(os.path.join(generator_proj, "hal", "include"), os.path.join(inc_dest, "hal"))
    shutil.copytree(os.path.join(generator_proj, "hal", "src"), os.path.join(src_dest, "hal"))
    shutil.copytree(os.path.join(generator_proj, "hal", "utils", "include"), os.path.join(inc_dest, "hal", "utils"))
    shutil.copytree(os.path.join(generator_proj, "hal", "utils", "src"), os.path.join(src_dest, "hal", "utils"))

    # hpl
    hpl_includes = []
    for name in os.listdir(os.path.join(generator_proj, "hpl")):
        full_path = os.path.join(generator_proj, "hpl", name)
        if os.path.isdir(full_path):
            inc_dest_dir = os.path.join(inc_dest, "hpl", name)
            src_dest_dir = os.path.join(src_dest, "hpl", name)
            inc_glb = "{}/**/*.h".format(full_path.replace("\\", "/"))
            src_glb = "{}/**/*.c".format(full_path.replace("\\", "/"))
            copyglob(inc_glb, inc_dest_dir)
            copyglob(src_glb, src_dest_dir)
            if os.path.exists(inc_dest_dir):
                hpl_includes.append(name)
    with open(os.path.join(inc_dest, "hpl_includes.txt"), 'w') as f:
        for item in hpl_includes:
            f.write("-Iinclude/v1_generated/hpl/{}\n".format(item))

    # hri
    shutil.copytree(os.path.join(generator_proj, "hri"), os.path.join(inc_dest, "hri"))

    # atmel_start
    shutil.copy(os.path.join(generator_proj, "atmel_start.h"), os.path.join(inc_dest))
    shutil.copy(os.path.join(generator_proj, "atmel_start_pins.h"), os.path.join(inc_dest))
    shutil.copy(os.path.join(generator_proj, "atmel_start.c"), os.path.join(src_dest))

    # driver_init
    shutil.copy(os.path.join(generator_proj, "driver_init.h"), os.path.join(inc_dest))
    shutil.copy(os.path.join(generator_proj, "driver_init.c"), os.path.join(src_dest))

    # Fix a delay bug due to wrong alignment if not using SysTick based delay driver
    # This bug can result in 2x to 44x delay times
    # See https://www.avrfreaks.net/s/topic/a5C3l000000UZLqEAO/t149684
    using_systick_delay = False
    with open(os.path.join(src_dest, "driver_init.c"), 'r') as f:
        line = f.readline()
        while line is not None and line != "":
            if line.find("delay_driver_init") != -1:
                using_systick_delay = True
                break
            line = f.readline()
    if not using_systick_delay:
        print("Detected ASF4 without SysTick delay driver. Patching hpl_core_m4.c to fix a bug.")
        contents = ""
        with open(os.path.join(src_dest, "hpl", "core", "hpl_core_m4.c"), 'r') as f:
            contents = f.read()
        contents = contents.replace("uint32_t _get_cycles_for_us(", "__attribute__ (( aligned(8) )) uint32_t _get_cycles_for_us(")
        contents = contents.replace("uint32_t _get_cycles_for_ms(", "__attribute__ (( aligned(8) )) uint32_t _get_cycles_for_ms(")
        with open(os.path.join(src_dest, "hpl", "core", "hpl_core_m4.c"), 'w') as f:
            f.write(contents)


    print("*** MANUALLY COPY INCLUDES TO FROM hpl_includes.txt to platformio.ini ***")


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
cmake_minimum_required(VERSION 3.20.0)
project(ControlBoard C ASM)


## Remove items from a list if they contain a specific substring
function (remove_items_containing items substring)
    foreach (TMP_PATH ${${items}})
        string (FIND ${TMP_PATH} "${substring}" EXCLUDE_DIR_FOUND)
        if (NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)
            list (REMOVE_ITEM ${items} ${TMP_PATH})
        endif ()
    endforeach(TMP_PATH)
    set(${items} ${${items}} PARENT_SCOPE)
endfunction(remove_items_containing)


####################################################################################################
# General Configuration
####################################################################################################

# Includes for all targets
set(INCLUDES 
    "${PROJECT_SOURCE_DIR}/include" 
    "${PROJECT_SOURCE_DIR}/thirdparty/FreeRTOS/include"
    "${PROJECT_SOURCE_DIR}/thirdparty/TinyUSB/"
)

# General sources (all targets)
file(GLOB_RECURSE SOURCES 
    "${PROJECT_SOURCE_DIR}/src/*.c"
    "${PROJECT_SOURCE_DIR}/thirdparty/FreeRTOS/*.c"
    "${PROJECT_SOURCE_DIR}/thirdparty/TinyUSB/*.c"
)
remove_items_containing(SOURCES "${PROJECT_SOURCE_DIR}/thirdparty/FreeRTOS/portable")
remove_items_containing(SOURCES "${PROJECT_SOURCE_DIR}/thirdparty/TinyUSB/portable")

# General preprocessor definitions (all targets)
set(DEFINES  "")

# General compile flags (all targets)
set(CFLAGS "")

# General linker flags (all targets)
set(LDFLAGS
    -Wl,--print-memory-usage
)


####################################################################################################
# Version Specific Configuration
####################################################################################################

if("${CBOARD_REV}" STREQUAL "v1")
    ################################################################################################
    # Control Board v1
    ################################################################################################

    # Version specific sources
    file(GLOB_RECURSE EXTRA_SOURCES 
        "${PROJECT_SOURCE_DIR}/thirdparty/v1_generated/*.c"
        "${PROJECT_SOURCE_DIR}/thirdparty/FreeRTOS/portable/GCC/ARM_CM4F/port.c"
        "${PROJECT_SOURCE_DIR}/thirdparty/FreeRTOS/portable/MemMang/heap_4.c"
        "${PROJECT_SOURCE_DIR}/thirdparty/TinyUSB/portable/microchip/samd-newdfp/dcd_samd.c"
    )

    # Version specific includes
    set(EXTRA_INCLUDES
        "${PROJECT_SOURCE_DIR}/thirdparty/v1_generated/mcc"
        "${PROJECT_SOURCE_DIR}/thirdparty/v1_generated/packs/CMSIS/CMSIS/Core/Include"
        "${PROJECT_SOURCE_DIR}/thirdparty/v1_generated/packs/ATSAMD51G19A_DFP"
        "${PROJECT_SOURCE_DIR}/thirdparty/FreeRTOS/portable/GCC/ARM_CM4F"
    )

    # Target specific preprocessor definitions
    set(EXTRA_DEFINES
        CONTROL_BOARD_V1
        CFG_TUSB_MCU=OPT_MCU_SAMD51
        __SAMD51G19A__
    )

    # Target specific flags (compilers and linke)
    set(EXTRA_SHAREDFLAGS
        # CPU and FPU config
        -mcpu=cortex-m4
        -mthumb
        -mthumb-interwork
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16

        # Newlib configuration
        --specs=nano.specs
        --specs=nosys.specs
    )

    # Target specific c commpiler flags
    set(EXTRA_CFLAGS
        ${EXTRA_SHAREDFLAGS}
        # -ffunction-sections
        # -fdata-sections
        # -mlong-calls
        -Wall
    )

    # Target specific linker flags
    set(EXTRA_LDFLAGS
        ${EXTRA_SHAREDFLAGS}
        # -Wl,--gc-sections
        -Wl,--print-memory-usage
        -T${PROJECT_SOURCE_DIR}/thirdparty/v1_generated/samd51g19a_flash.ld
    )
    
    ################################################################################################
elseif("${CBOARD_REV}" STREQUAL "v2")
    ################################################################################################
    # Control Board v2
    ################################################################################################
    
    # Version specific sources
    file(GLOB_RECURSE EXTRA_SOURCES 
        "${PROJECT_SOURCE_DIR}/thirdparty/v2_generated/*.c"
        "${PROJECT_SOURCE_DIR}/thirdparty/v2_generated/startup_stm32f411xe.s"
        "${PROJECT_SOURCE_DIR}/thirdparty/FreeRTOS/portable/GCC/ARM_CM4F/port.c"
        "${PROJECT_SOURCE_DIR}/thirdparty/FreeRTOS/portable/MemMang/heap_4.c"
        "${PROJECT_SOURCE_DIR}/thirdparty/TinyUSB/portable/synopsys/dwc2/dcd_dwc2.c"
    )

    # Version specific includes
    set(EXTRA_INCLUDES
        "${PROJECT_SOURCE_DIR}/thirdparty/v2_generated/Core/Inc"
        "${PROJECT_SOURCE_DIR}/thirdparty/v2_generated/Drivers/CMSIS/Include"
        "${PROJECT_SOURCE_DIR}/thirdparty/v2_generated/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
        "${PROJECT_SOURCE_DIR}/thirdparty/v2_generated/Drivers/STM32F4xx_HAL_Driver/Inc"
        "${PROJECT_SOURCE_DIR}/thirdparty/FreeRTOS/portable/GCC/ARM_CM4F"
    )

    # Target specific preprocessor definitions
    set(EXTRA_DEFINES
        CONTROL_BOARD_V2
        CFG_TUSB_MCU=OPT_MCU_STM32F4
        STM32F411xE
    )

    # Target specific flags (compilers and linke)
    set(EXTRA_SHAREDFLAGS
        # CPU and FPU config
        -mcpu=cortex-m4
        -mthumb
        -mthumb-interwork
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16

        # Newlib configuration
        --specs=nano.specs
        --specs=nosys.specs
    )

    # Target specific c commpiler flags
    set(EXTRA_CFLAGS
        ${EXTRA_SHAREDFLAGS}
        # -ffunction-sections
        # -fdata-sections
        # -mlong-calls
        -Wall
    )

    # Target specific linker flags
    set(EXTRA_LDFLAGS
        ${EXTRA_SHAREDFLAGS}
        # -Wl,--gc-sections
        -T${PROJECT_SOURCE_DIR}/thirdparty/v2_generated/STM32F411CEUx_FLASH.ld
    )

    ################################################################################################
else()
    message(FATAL_ERROR "Invalid value specified for CBOARD_REV.")
endif()


####################################################################################################
# Executable Setup
####################################################################################################

add_executable(ControlBoard ${SOURCES} ${EXTRA_SOURCES})
set_property(TARGET ControlBoard PROPERTY C_STANDARD 11)
set_property(TARGET ControlBoard PROPERTY C_STANDARD_REQUIRED ON)
target_include_directories(ControlBoard PUBLIC ${INCLUDES} ${EXTRA_INCLUDES})
target_compile_definitions(ControlBoard PUBLIC ${DEFINES} ${EXTRA_DEFINES})
target_compile_options(ControlBoard PUBLIC ${CFLAGS} ${EXTRA_CFLAGS})
target_link_options(ControlBoard PUBLIC ${LDFLAGS} ${EXTRA_LDFLAGS})
target_link_libraries(ControlBoard m)
add_custom_command(TARGET ControlBoard POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -O binary "$<OUTPUT_CONFIG:$<CONFIG>>/ControlBoard.elf" "$<OUTPUT_CONFIG:$<CONFIG>>/ControlBoard.bin"
)
add_custom_command(TARGET ControlBoard POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -O ihex "$<OUTPUT_CONFIG:$<CONFIG>>/ControlBoard.elf" "$<OUTPUT_CONFIG:$<CONFIG>>/ControlBoard.hex"
)

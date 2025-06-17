# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_function_generator C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(${PROJECT_NAME})

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/i2c/i2c.pio)

# Add executable. Default name is the project name, version 0.1
target_sources(${PROJECT_NAME} PRIVATE
    drivers/mcp4725.hpp
    drivers/sh1106.hpp
    i2c/i2c_dma.c
    i2c/i2c_dma.h
    i2c/i2c_interface.hpp
    i2c/i2c_pico.cpp
    i2c/i2c_pico.hpp
    i2c/pio_i2c.c
    i2c/pio_i2c.h
    ui/ui_builder.cpp
    ui/ui_builder.hpp
    ui/ui_task.cpp
    ui/ui_task.hpp
    ui/ui_visitor.hpp
    waveform/waveform_data.hpp
    waveform/waveform_task.cpp
    waveform/waveform_task.hpp
    waveform/waveform_visitor.cpp
    waveform/waveform_visitor.hpp
    hal.hpp
    lv_conf.h
    main.cpp
)

add_subdirectory(lvgl)

# Include LVGL headers
include_directories(${CMAKE_SOURCE_DIR}/lvgl)

# Link LVGL to your target
target_link_libraries(${PROJECT_NAME} lvgl)

# pull in common dependencies and additional pwm hardware support
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    hardware_adc
    hardware_dma
    hardware_i2c
    hardware_pio
    hardware_pwm
    hardware_timer
)

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

FetchContent_Declare(freertos_kernel
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
    GIT_TAG V11.2.0 #Note: Best practice to use specific git-hash or tagged version
)

add_library(freertos_config INTERFACE)

target_include_directories(freertos_config SYSTEM
    INTERFACE
    include
)

target_compile_definitions(freertos_config
    INTERFACE
    projCOVERAGE_TEST=0
)

set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)
# Select the cross-compile PORT
if(CMAKE_CROSSCOMPILING)
    set(FREERTOS_PORT "GCC_RP2040" CACHE STRING "" FORCE)
endif()

FetchContent_MakeAvailable(freertos_kernel)

# Add the FreeRTOS kernel and configuration
target_link_libraries(${PROJECT_NAME} freertos_kernel)
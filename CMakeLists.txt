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
    i2c/i2c_interface.hpp
    i2c/i2c_pico.hpp
    i2c/pio_i2c.c
    i2c/pio_i2c.h
    hal.hpp
    lv_conf.h
    ui_task.hpp
    ui_task.cpp
    main.cpp
    mcp4725_Sinewave_Data.cpp
    mcp4725_Sinewave_Data.hpp
    mcp4725.hpp
    sh1106.hpp
    waveform_data.hpp
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

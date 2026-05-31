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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(hexapod_pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(hexapod_pico
    hexapod_pico.c
    Src/hexapod_core.c
    Src/hexapod_ik.c
    Src/hexapod_gait.c
    Src/hexapod_math.c
    Src/hexapod_hal_pico.c
    Src/hexapod_i2c_protocol.c
    Src/hexapod_crsf.c
)

pico_set_program_name(hexapod_pico "hexapod_pico")
pico_set_program_version(hexapod_pico "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(hexapod_pico 1)
pico_enable_stdio_usb(hexapod_pico 1)

# Add the standard library to the build
target_link_libraries(hexapod_pico
        pico_stdlib
        hardware_i2c
        hardware_adc
        hardware_uart
)

# Add the standard include files to the build
target_include_directories(hexapod_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    Inc
    Src
)

pico_add_extra_outputs(hexapod_pico)


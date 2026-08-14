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
    Src/hexapod_ps2.c
    Src/bno055.c
)

pico_set_program_name(hexapod_pico "hexapod_pico")
pico_set_program_version(hexapod_pico "0.1")

# Modify the below lines to enable/disable output over UART/USB
# 注意: stdio UART 默认占用 UART0 = GP0(TX)/GP1(RX),
# 而 PCB 定版中 GP0/GP1 预留给外部扩展 (上位机/传感器),
# 所以调试输出只走 USB CDC, 保持 GP0/GP1 空闲。
pico_enable_stdio_uart(hexapod_pico 0)
pico_enable_stdio_usb(hexapod_pico 1)

# Add the standard library to the build
# ★ 显式列出所有直接使用的 hardware_* 库 (勿依赖传递依赖, SDK 内部链条易变):
target_link_libraries(hexapod_pico
        pico_stdlib
        hardware_i2c        # I2C1: PCA9685 + BNO055
        hardware_adc        # GP26 电池电压
        hardware_uart       # UART1: CRSF/串口
        hardware_pwm        # 无源蜂鸣器 GP13 + 直流电机 GP2/GP3
        hardware_gpio       # 足端开关/供电控制/LED/PS2 bit-bang
        hardware_sync       # save_and_disable_interrupts (I2C 临界区)
        hardware_watchdog   # 主循环硬件看门狗
        hardware_irq        # UART1 中断处理 (irq_set_exclusive_handler)
)

# Add the standard include files to the build
target_include_directories(hexapod_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    Inc
    Src
)

pico_add_extra_outputs(hexapod_pico)


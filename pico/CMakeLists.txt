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
    Src/hexapod_input.c
    Src/bno055.c
    Src/hexapod_store.c
    Src/hexapod_params.c
)

pico_set_program_name(hexapod_pico "hexapod_pico")
# picotool 元数据用的版本 (要 MAJOR.MINOR.REVISION 形式, 与 git tag 对齐);
# 板上显示的版本另取 git describe, 见下面的 HEXAPOD_FW_GIT
pico_set_program_version(hexapod_pico "0.4.0")

# 固件版本: 每次构建重新生成 hexapod_version.h (git describe: tag + 短哈希 +
# dirty 标记)。烧录后网页/串口一看就知道板上跑的到底是不是刚编的那次。
set(HEXAPOD_VERSION_H ${CMAKE_CURRENT_BINARY_DIR}/hexapod_version.h)
# 本地构建留空 = 按 git describe 走 (能看出 dirty); 发布工作流传 tag 名 (见 release.yml),
# 不去猜 CI checkout 里刚推的 tag 在不在
set(HEXAPOD_FW_GIT_OVERRIDE "" CACHE STRING "直接指定固件版本串, 空 = 用 git describe")
add_custom_target(hexapod_gen_version
        COMMAND ${CMAKE_COMMAND}
                -DSRC_DIR=${CMAKE_CURRENT_LIST_DIR}
                -DOUT=${HEXAPOD_VERSION_H}
                -DHEXAPOD_FW_GIT_OVERRIDE=${HEXAPOD_FW_GIT_OVERRIDE}
                -P ${CMAKE_CURRENT_LIST_DIR}/cmake/gen_version.cmake
        BYPRODUCTS ${HEXAPOD_VERSION_H}
        COMMENT "生成固件版本头 (版本串: git describe 或 HEXAPOD_FW_GIT_OVERRIDE)")
add_dependencies(hexapod_pico hexapod_gen_version)

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
        hardware_flash      # 非易失存储: flash_range_erase/program (hexapod_store.c)
)

# Add the standard include files to the build
target_include_directories(hexapod_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    Inc
    Src
    ${CMAKE_CURRENT_BINARY_DIR}    # hexapod_version.h (构建时生成)
)

pico_add_extra_outputs(hexapod_pico)


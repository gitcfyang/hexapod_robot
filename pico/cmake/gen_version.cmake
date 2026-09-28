# 生成固件版本头 (由 CMakeLists.txt 的 add_custom_target 每次构建调用)
#
# 用法: cmake -DSRC_DIR=<仓库 pico 目录> -DOUT=<生成的 .h 路径>
#            -DHEXAPOD_FW_GIT_OVERRIDE=<版本串, 可空> -P gen_version.cmake
#
# 为什么不用 configure 期的 execute_process: 那样只在 cmake configure 时取一次
# git describe, 之后改了代码重新 make 仍然报旧哈希 —— 烧录后版本没变会被误判成
# "没换上"。这里每次构建都重跑, 版本串必然对应本次编译的源码。
#
# HEXAPOD_FW_GIT_OVERRIDE 非空时直接当版本串用, 不查 git: 发布工作流发的就是它
# 正在打的那个 tag, 不必也不该去猜 CI checkout 的 tag 状态 —— 2026-09-28 实测,
# v0.4.0 那次 release 里 fetch-depth: 0 也没让 git describe 看见刚推的 v0.4.0
# (老 tag 在), 编出来的 uf2 报的是 v0.3.0-56-g9be97c7。这个串会显示在网页/串口的
# 启动横幅上, 是"板上跑的到底是不是刚发的这版"的凭据, 不能靠运气。
if(HEXAPOD_FW_GIT_OVERRIDE)
    set(HEXAPOD_FW_GIT "${HEXAPOD_FW_GIT_OVERRIDE}")
else()
    execute_process(
        COMMAND git describe --always --dirty
        WORKING_DIRECTORY "${SRC_DIR}"
        OUTPUT_VARIABLE HEXAPOD_FW_GIT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
endif()

if(NOT HEXAPOD_FW_GIT)
    set(HEXAPOD_FW_GIT "unknown")   # 源码包 (无 .git) 或无 git 命令
endif()

file(WRITE "${OUT}"
     "/* 自动生成, 勿手改 —— 由 pico/cmake/gen_version.cmake 在构建时写入 */\n"
     "#define HEXAPOD_FW_VERSION \"${HEXAPOD_FW_GIT}\"\n")

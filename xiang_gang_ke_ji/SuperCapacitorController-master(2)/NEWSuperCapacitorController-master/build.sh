#!/bin/bash
# STM32 SuperCapacitor Controller 编译脚本 (使用 Ninja)
# ========================================
# 💡 在这里修改硬件 ID（有效值: 100, 101, 102, 201, 202, 203, 204, 205）
# ========================================
DEFAULT_HARDWARE_ID=101
# ========================================

# 使用方法: ./build.sh [HARDWARE_ID]
# 示例: 
#   ./build.sh       # 使用默认 ID (101)
#   ./build.sh 202   # 使用指定 ID (202)

# 设置 HARDWARE_ID (命令行参数优先，否则使用默认值)
HARDWARE_ID=${1:-$DEFAULT_HARDWARE_ID}

echo "=========================================="
echo "编译 STM32 超级电容控制器 (Ninja)"
echo "HARDWARE_ID: $HARDWARE_ID"
echo "=========================================="

# 检查 HARDWARE_ID 是否有效
# 100 = 模板（用于校准）
# 101-102 = 第一批硬件
# 201-205 = 第二批硬件
VALID_IDS="100 101 102 201 202 203 204 205"
if [[ ! " $VALID_IDS " =~ " $HARDWARE_ID " ]]; then
    echo "错误: 无效的 HARDWARE_ID: $HARDWARE_ID"
    echo "有效的 HARDWARE_ID: $VALID_IDS"
    echo ""
    echo "说明:"
    echo "  100     - 校准模板"
    echo "  101-102 - 第一批硬件"
    echo "  201-205 - 第二批硬件"
    exit 1
fi

# 检查 ARM 工具链是否已安装
if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo "错误: 未找到 ARM GCC 工具链 (arm-none-eabi-gcc)"
    echo "请安装 ARM GCC 工具链"
    exit 1
fi

# 检查 cmake
if ! command -v cmake &> /dev/null; then
    echo "错误: 未找到 cmake"
    echo "请安装 CMake"
    exit 1
fi

# 检查 ninja
if ! command -v ninja &> /dev/null; then
    echo "错误: 未找到 ninja 构建工具"
    echo "请安装 Ninja: https://ninja-build.org/"
    exit 1
fi

echo "使用构建工具: Ninja"
echo "ARM GCC 版本:"
arm-none-eabi-gcc --version | head -n 1
echo "CMake 版本:"
cmake --version | head -n 1
echo "Ninja 版本:"
ninja --version

# 创建临时 CMakeLists.txt
echo ""
echo "生成构建配置..."
cat > CMakeLists.txt.tmp << 'EOF'
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(RM2024-SuperCap-F3-V1R C CXX ASM)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)

# HARDWARE_ID can be set via command line
if(NOT DEFINED HARDWARE_ID)
    set(HARDWARE_ID 101)
endif()

message(STATUS "Building with HARDWARE_ID=${HARDWARE_ID}")

# Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

# MCU settings
add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_compile_options(-fstack-usage -specs=nano.specs)

# Warnings
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion)
add_compile_options(-Wlogical-op -Wpointer-arith -Wmissing-field-initializers)
add_compile_options(-Wno-unused-parameter -Wno-unused-const-variable)
add_compile_options(-Wfatal-errors -fdiagnostics-color=auto)

# Optimization
add_compile_options(-O3 -g3 -gdwarf-4)

# Include directories
include_directories(
    Core/Inc
    Drivers/STM32F3xx_HAL_Driver/Inc
    Drivers/STM32F3xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F3xx/Include
    Drivers/CMSIS/Include
    Middlewares/ST/ARM/DSP/Inc
)

# Definitions
add_definitions(
    -DUSE_HAL_DRIVER
    -DSTM32F334x8
    -DARM_MATH_CM4
    -DHARDWARE_ID=${HARDWARE_ID}
)

# C source files
set(C_SOURCES
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/adc.c
    Core/Src/can.c
    Core/Src/hrtim.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32f3xx_it.c
    Core/Src/stm32f3xx_hal_msp.c
    Core/Src/system_stm32f3xx.c
    Core/Src/dma.c
    Core/Src/iwdg.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_i2c_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_exti.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_can.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_hrtim.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c
    Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_iwdg.c
)

# CPP source files
file(GLOB CPP_SOURCES "Core/Src/*.cpp")

# ASM source files
set(ASM_SOURCES
    startup_stm32f334x8_SuperCap.s
)

# Linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F334R8Tx_SuperCap_FLASH.ld)

# C++ specific flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti -fno-threadsafe-statics")

# Linker options
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})
add_link_options(-specs=nano.specs -lc -lm -lnosys)

# Create executable
add_executable(${PROJECT_NAME}.elf ${C_SOURCES} ${CPP_SOURCES} ${ASM_SOURCES} ${LINKER_SCRIPT})

# Generate hex and bin files
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
    COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}")

# Print size
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${SIZE} ${PROJECT_NAME}.elf
    COMMENT "")
EOF

# 清理旧的构建文件
echo ""
echo "清理旧的构建文件..."
rm -rf build

# 使用临时 CMakeLists.txt
mv CMakeLists.txt.tmp CMakeLists.txt

# 配置 CMake
echo ""
echo "配置 CMake (HARDWARE_ID=$HARDWARE_ID)..."
cmake -B build -G Ninja -DHARDWARE_ID=$HARDWARE_ID

CMAKE_STATUS=$?
if [ $CMAKE_STATUS -ne 0 ]; then
    echo ""
    echo "=========================================="
    echo "❌ CMake 配置失败！"
    echo "=========================================="
    rm -f CMakeLists.txt
    exit 1
fi

# 编译项目
echo ""
echo "开始编译..."
ninja -C build

BUILD_STATUS=$?

# 清理临时文件
rm -f CMakeLists.txt

if [ $BUILD_STATUS -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✅ 编译成功！"
    echo "=========================================="
    echo "输出文件位于 build/ 目录:"
    echo "  - RM2024-SuperCap-F3-V1R.elf"
    echo "  - RM2024-SuperCap-F3-V1R.hex"
    echo "  - RM2024-SuperCap-F3-V1R.bin"
    echo "=========================================="
    
    # 显示文件大小
    if [ -f build/RM2024-SuperCap-F3-V1R.elf ]; then
        echo ""
        echo "固件文件大小:"
        ls -lh build/RM2024-SuperCap-F3-V1R.{elf,hex,bin} 2>/dev/null | awk '{print "  " $9 ": " $5}'
    fi
else
    echo ""
    echo "=========================================="
    echo "❌ 编译失败！"
    echo "=========================================="
    exit 1
fi

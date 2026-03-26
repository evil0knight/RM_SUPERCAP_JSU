# 编译指南

## 🔧 环境要求

1. **ARM GCC 工具链** - `arm-none-eabi-gcc`
2. **CMake** - 版本 3.22 或更高
3. **Ninja** - 构建工具
4. **Git Bash** - Windows 环境下需要

## 📝 在 Git Bash 中编译

### 1. 打开 Git Bash

在项目目录中右键，选择 "Git Bash Here"

### 2. 给脚本添加执行权限（首次运行）

```bash
chmod +x build.sh
```

### 3. 运行编译脚本

```bash
# 使用默认 HARDWARE_ID (101)
./build.sh

# 指定 HARDWARE_ID
./build.sh 202
```

## 🆔 支持的 HARDWARE_ID

| ID | 类型 | 说明 |
|----|------|------|
| 100 | 校准模板 | 用于硬件校准 |
| 101-102 | 第一批硬件 | 已校准的第一批硬件 |
| 201-205 | 第二批硬件 | 已校准的第二批硬件 |

## ⚠️ 常见问题

### 问题1: 提示找不到 bash

**原因**: 在 PowerShell 中运行了脚本

**解决**: 在 Git Bash 中运行，而不是 PowerShell

### 问题2: 提示找不到 cmake 或 ninja

**原因**: 未安装或未添加到 PATH

**解决**: 
1. 下载安装 CMake: https://cmake.org/download/
2. 下载安装 Ninja: https://github.com/ninja-build/ninja/releases
3. 将它们的 bin 目录添加到系统 PATH

### 问题3: 提示找不到 arm-none-eabi-gcc

**原因**: ARM 工具链未安装或未添加到 PATH

**解决**: 
检查工具链是否在 PATH 中：
```bash
which arm-none-eabi-gcc
```

如果没有输出，需要将工具链目录添加到 PATH

### 问题4: CMake 配置失败

**原因**: CMake 版本过低或配置错误

**解决**: 
```bash
# 检查 CMake 版本
cmake --version

# 确保版本 >= 3.22
```

## 🎯 编译输出

编译成功后，输出文件位于 `build/` 目录：
- `RM2024-SuperCap-F3-V1R.elf` - ELF 可执行文件
- `RM2024-SuperCap-F3-V1R.hex` - HEX 固件文件
- `RM2024-SuperCap-F3-V1R.bin` - BIN 固件文件

## 🔄 清理构建文件

```bash
rm -rf build
```

## 💡 提示

- 脚本会自动创建临时的 CMakeLists.txt
- 编译完成后会自动删除临时文件
- 不会修改项目的原始文件




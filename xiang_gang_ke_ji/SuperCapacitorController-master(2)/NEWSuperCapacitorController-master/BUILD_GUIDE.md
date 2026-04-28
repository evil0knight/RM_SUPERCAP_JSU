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
# 指定 HARDWARE_ID
./build.sh 101
```

## 🆔 支持的 HARDWARE_ID

| ID  | 类型       | 说明               |
| --- | ---------- | ------------------ |
| 100 | 校准模板   | 用于硬件校准       |
| 102 | 第一批硬件 | 已校准的第一批硬件 |
| 101 | 第二批硬件 | 已校准的第二批硬件 |

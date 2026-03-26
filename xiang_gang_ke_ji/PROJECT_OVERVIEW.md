# RM2024 超级电容控制器 - 项目理解文档

> 本文档用于快速理解项目架构、核心模块和开发流程
>
> 最后更新：2025-11-03

---

## 1. 项目基本信息

### 1.1 项目简介

- **项目名称**: RM2024 超级电容控制器
- **开发团队**: 香港科技大学 ENTERPRIZE 战队
- **设计者**: 蒋益诚、戎羿
- **目标平台**: STM32F334R8
- **开源协议**: GPL 3.0
- **核心功能**: 超级电容充放电管理、功率分配、安全保护

### 1.2 硬件规格

- **MCU**: STM32F334R8
  - Flash: 64 KB
  - RAM: 12 KB
  - 主频: 72 MHz
  - 特色外设: HRTIM (高分辨率定时器)
- **电压范围**:
  - 输入电压 (A侧): 15V ~ 30V
  - 电容电压 (B侧): 0V ~ 28.8V
- **电流能力**: 最大 22.5A

笔记  
---本项目采用RY8411隔离降压电源,给下板栅极供电,隔离电源更稳定  
    电压采集是通过电阻分压(/10)然后到电压跟随器,电压跟随器的作用是增加输入阻抗输入端开路,输出端电流大,起隔离作用,常用于ADC  

## 2. 系统架构

### 2.1 硬件架构


```
裁判系统 (24V输入)
    |
    ├─→ [控制板] STM32F334R8
    |      ├─ ADC: 电压电流采样
    |      ├─ HRTIM: 高精度PWM输出
    |      ├─ CAN: 通信接口
    |      └─ UART: 调试接口
    |
    ├─→ [功率板] Buck-Boost 电路
    |      ├─ A侧: 裁判系统接口
    |      └─ B侧: 超级电容接口
    |
    └─→ 底盘 (功率输出)
```

### 2.2 软件架构

```
main.c
  └─→ systemStart() [SuperCap.cpp]
       ├─→ 硬件ID检查
       ├─→ Buzzer::init()
       ├─→ Communication::init()
       ├─→ PowerManager::init()
       │     ├─→ ADC校准和启动
       │     ├─→ HRTIM配置和启动
       │     ├─→ 定时器配置
       │     └─→ 硬件错误检查
       └─→ loop() + IWDG看门狗
```

### 2.3 中断架构

| 中断                         | 频率   | 功能                     |
| ---------------------------- | ------ | ------------------------ |
| `HRTIM1_Master_IRQHandler` | ~20kHz | V/I控制循环（核心）      |
| `TIM2_IRQHandler`          | 1kHz   | 状态更新、错误检查、通信 |

---

## 3. 核心模块详解

### 3.1 PowerManager (电源管理器)

**文件位置**: [Core/Src/PowerManager.cpp](Core/Src/PowerManager.cpp:1)

#### 3.1.1 命名空间结构

```cpp
namespace PowerManager {
    // 主要数据结构
    struct ControlData;       // 控制输入数据
    struct Status;            // 状态输出数据

    namespace TimerManager {  // HRTIM PWM控制
        void HRTIMStartPWM();
        void HRTIMEnableOutput();
        void HRTIMDisableOutput();
    }

    namespace SampleManager { // ADC采样和数据处理
        struct ADCSample;
        struct ProcessedData;
        void updateAndCalibrate();
    }

    namespace ErrorChecker {  // 错误检查和保护
        bool checkHardwareError();
        void handleShortCircuit();
        void handleErrorState();
    }
}
```

#### 3.1.2 关键数据结构

**ControlData** (控制输入)

```cpp
struct ControlData {
    float refereePowerLimit;  // 裁判系统功率限制
    float energyRemain;       // 剩余能量缓冲
    bool enableOutput;        // 输出使能
};
```

**Status** (状态输出)

```cpp
struct Status {
    bool outputEnabled;       // 输出状态
    uint8_t errorCode;        // 错误代码
    float capEnergy;          // 电容能量 (归一化)
    float chassisPower;       // 底盘功率
    float chassisPowerLimit;  // 底盘功率限制
    bool lowBattery;          // 低电压警告
    float realVBToVA;         // 实际电压比
};
```

**ProcessedData** (处理后的采样数据)

```cpp
struct ProcessedData {
    float vASide, iASide;     // A侧电压电流
    float vBSide, iBSide;     // B侧电压电流
    float iReferee, pReferee; // 裁判系统电流功率
    float iChassis;           // 底盘电流
    float pASide, pBSide;     // A/B侧功率
    float efficiency;         // 转换效率
    float temperature;        // 温度
    float capCharge;          // 电容累积电荷
};
```

#### 3.1.3 PID控制器

| PID控制器            | 控制目标     | 参数 (Kp, Ki, Kd, Limit)         |
| -------------------- | ------------ | -------------------------------- |
| `pidVoltageB`      | 电容电压     | (1.00, 2.00, 0.05, 0.7)          |
| `pidCurrentA`      | A侧电流      | (0.0046, 0.0091, 0.0015, 0.0015) |
| `pidPowerReferee`  | 裁判系统功率 | (0.205, 0.31, 0.03, 0.022)       |
| `pidEnergyReferee` | 能量缓冲     | (0.5, 2.0, 0.5, 0.2)             |

---

### 3.2 通信模块 (Communication)

**文件位置**: [Core/Src/Communication.cpp](Core/Src/Communication.cpp:1)

- **CAN通信**: 接收裁判系统数据（功率限制、能量缓冲）
- **反馈数据**: 发送电容状态、底盘功率等信息

---

### 3.3 Buck-Boost 控制

**文件位置**: [PowerManager.cpp:637](Core/Src/PowerManager.cpp:637)

#### 3.3.1 工作模式

| 模式                 | 电压比 VB/VA | DutyA              | DutyB              |
| -------------------- | ------------ | ------------------ | ------------------ |
| **Buck模式**   | < 0.9        | 0.9 × (VB/VA)     | 0.9                |
| **Boost模式**  | > 1.111      | 0.9                | 0.9 / (VB/VA)      |
| **Buck-Boost** | 0.9 ~ 1.111  | (VB/VA + 1) × 0.4 | (VA/VB + 1) × 0.4 |

#### 3.3.2 PWM输出

- **HRTIM频率**: ~140kHz (HRTIM_PERIOD = 16000)
- **PWM模式**: 互补对称PWM
- **分辨率**: 4.4ns (高分辨率定时器特性)

---

### 3.4 错误检查和保护

**文件位置**: [PowerManager.cpp:191](Core/Src/PowerManager.cpp:191)

#### 3.4.1 错误代码定义

| 错误代码                   | 值   | 含义           | 触发条件                |
| -------------------------- | ---- | -------------- | ----------------------- |
| `ERROR_UNDER_VOLTAGE`    | 0x01 | 欠压           | VA < 15V 或 VB < 12V    |
| `ERROR_OVER_VOLTAGE`     | 0x02 | 过压           | VA > 31V 或 VB > 31V    |
| `ERROR_BUCK_BOOST`       | 0x04 | Buck-Boost故障 | 效率 < 50% 或占空比异常 |
| `ERROR_SHORT_CIRCUIT`    | 0x08 | 短路           | V < 4V 且 I > 15.5A     |
| `ERROR_HIGH_TEMPERATURE` | 0x10 | 过温           | 温度 > 180 ADC值        |
| `ERROR_NO_POWER_INPUT`   | 0x20 | 无输入电源     | VA < 15V                |
| `ERROR_CAPACITOR`        | 0x40 | 电容故障       | 电容断开或短路          |

#### 3.4.2 保护策略

1. **短路保护**: 检测到短路后立即关闭输出，500ms冷却
2. **过压保护**: 累积计数触发，防止误触发
3. **电容检测**: 通过电压变化率和充电量估算电容值
4. **Buck-Boost检测**: 监测效率和占空比异常
5. **温度保护**: 带迟滞的温度保护

---

## 4. 控制算法

### 4.1 多级串级控制

```
能量缓冲 PID
    ↓
[baseRefereePower] ← 功率限制变化补偿
    ↓
裁判系统功率 PID
    ↓
[targetAPower] ← 电流限制约束
    ↓
A侧电流 PID + 积分补偿
    ↓
[outputDuty] ← 电容电压 PID 限幅
    ↓
Buck-Boost PWM输出
```

### 4.2 前馈控制

- **电压前馈**: `targetIA = targetAPower / vASide`
- **占空比前馈**: 初始占空比基于电压比

### 4.3 电流限制策略

```cpp
// A侧电流限制 (受A侧和B侧共同约束)
powerLimitAToB = min(
    I_LIMIT * vASide,              // A侧硬件限制
    capInILimit * vASide * ratio   // B侧充电限制
);

powerLimitBToA = max(
    -I_LIMIT * vASide,             // A侧硬件限制
    -capOutILimit * vASide * ratio // B侧放电限制
);
```

**电容放电限制曲线**:

- VB < 5V: 最小放电 0.1A
- 5V < VB < 12V: 线性增加
- VB > 12V: 最大放电 22.5A

---

## 5. 数据流图

### 5.1 高速循环 (~20kHz)

```
ADC DMA采样 (双ADC并行)
    ↓
SampleManager::sumBuffer()       // 求和8次过采样
    ↓
ProcessedData::updateAndCalibrate() // 校准 + 滤波
    ↓
ErrorChecker::handleShortCircuit()  // 短路检查
    ↓
updateVIP()                      // 控制算法
    ├─ pidPowerReferee           // 功率PID
    ├─ 电流限制计算
    ├─ pidCurrentA               // 电流PID
    └─ pidVoltageB (条件)        // 电压PID
    ↓
updatePWM()                      // 更新PWM占空比
```

### 5.2 低速循环 (1kHz)

```
updateStatus()                   // 状态更新
    ├─ 计算电容能量
    ├─ 计算底盘功率
    ├─ 通信超时检查
    ├─ 温度采样
    └─ LED指示更新
    ↓
ErrorChecker::handleErrorState() // 错误状态机
    ├─ 欠压/过压检查
    ├─ 过温检查
    ├─ 电容故障检查
    ├─ Buck-Boost故障检查
    └─ 输出控制
    ↓
Communication::feedbackPowerData() // 发送反馈数据
```

---

## 6. 配置参数

**文件位置**: [Core/Inc/Config.hpp](Core/Inc/Config.hpp:1)

| 参数                      | 值     | 说明                |
| ------------------------- | ------ | ------------------- |
| `MAX_CAP_VOLTAGE`       | 28.8V  | 电容最大电压        |
| `I_LIMIT`               | 22.5A  | 最大电流限制        |
| `SHORT_CIRCUIT_CURRENT` | 15.5A  | 短路检测电流阈值    |
| `SHORT_CIRCUIT_VOLTAGE` | 4.0V   | 短路检测电压阈值    |
| `LOW_BATTERY_VOLTAGE`   | 20.92V | 低电压警告阈值      |
| `DEFAULT_REFEREE_POWER` | 37.0W  | 默认裁判功率        |
| `ENERGY_BUFFER`         | 50.0J  | 能量缓冲目标        |
| `ADC_SAMPLE_COUNT`      | 8      | ADC过采样倍数       |
| `ADC_FILTER_ALPHA`      | 0.5    | 电流滤波系数        |
| `HRTIM_PERIOD`          | 16000  | HRTIM周期 (~140kHz) |

---

## 7. 编译和开发

### 7.1 编译命令

```bash
# 必须指定 HARDWARE_ID
make -j HARDWARE_ID=101

# 清理
make clean

# 重新编译
make rebuild HARDWARE_ID=101
```

### 7.2 烧录方式

**方式1: ST-Link + STM32CubeProgrammer**

```bash
STM32_Programmer_CLI.exe -c port=SWD -e all -d build/RM2024-SuperCap-F3-V1R.hex -v -hardRst
```

**方式2: JLink + Ozone (推荐调试)**

- 双击 `SuperCap-F334.jdebug`
- 按 F5 下载并运行

### 7.3 编译模式

在编译时可以定义以下宏：

| 宏                         | 说明                   |
| -------------------------- | ---------------------- |
| `CALIBRATION_MODE`       | 校准模式，跳过硬件检查 |
| `IGNORE_CAPACITOR_ERROR` | 忽略电容错误检测       |
| `HARDWARE_ID=xxx`        | 硬件ID（必需）         |

---

## 8. 调试技巧

### 8.1 关键变量监视

```cpp
// 控制数据
PowerManager::ControlData::controlData.refereePowerLimit
PowerManager::ControlData::controlData.energyRemain

// 状态数据
PowerManager::Status::status.errorCode
PowerManager::Status::status.capEnergy
PowerManager::Status::status.chassisPower

// 采样数据
PowerManager::SampleManager::ProcessedData::processedData.vASide
PowerManager::SampleManager::ProcessedData::processedData.iASide
PowerManager::SampleManager::ProcessedData::processedData.vBSide
PowerManager::SampleManager::ProcessedData::processedData.iBSide

// 控制变量
tempData.outputDuty
tempData.targetAPower
tempData.targetIA
```

### 8.2 Ozone实时采样

1. View → Data Sampling
2. 添加变量（如 `processedData.vASide`）
3. 点击 Start
4. 查看实时波形

### 8.3 蜂鸣器状态指示

| 频率   | 时长   | 含义              |
| ------ | ------ | ----------------- |
| 800Hz  | 500ms  | 系统启动          |
| 1000Hz | 持续   | HARDWARE_ID不匹配 |
| 1250Hz | 200ms  | 输出使能          |
| 200Hz  | 100ms  | 短路保护触发      |
| 200Hz  | 200ms  | 错误状态          |
| 2700Hz | 1000ms | 硬件故障          |
| 3200Hz | 1600ms | 低电压警告        |

---

## 9. 常见问题

### 9.1 蜂鸣器持续响 1000Hz

- **原因**: HARDWARE_ID 不匹配
- **解决**:
  1. 确认编译时使用的 `HARDWARE_ID`
  2. 在校准模式下重新烧录

### 9.2 输出无法使能

- **检查点**:
  1. `errorCode` 是否为 0
  2. `enableOutput` 是否为 true
  3. 电压是否在正常范围 (VA > 15V)
  4. 是否通过硬件检查

### 9.3 效率异常

- **可能原因**:
  1. MOSFET损坏
  2. 驱动电路异常
  3. 电感饱和
  4. 电流采样异常

---

## 10. 项目文件导航

### 10.1 核心代码文件

| 文件                                           | 行数 | 功能           |
| ---------------------------------------------- | ---- | -------------- |
| [PowerManager.cpp](Core/Src/PowerManager.cpp)     | 1013 | 电源管理核心   |
| [SuperCap.cpp](Core/Src/SuperCap.cpp)             | 107  | 系统初始化入口 |
| [Communication.cpp](Core/Src/Communication.cpp)   | ~150 | CAN通信        |
| [IncrementalPID.cpp](Core/Src/IncrementalPID.cpp) | ~60  | PID控制器      |
| [Buzzer.cpp](Core/Src/Buzzer.cpp)                 | ~20  | 蜂鸣器控制     |

### 10.2 配置和头文件

| 文件                                       | 功能                 |
| ------------------------------------------ | -------------------- |
| [Config.hpp](Core/Inc/Config.hpp)             | 系统参数配置         |
| [calibration.hpp](Core/Inc/calibration.hpp)   | ADC校准参数          |
| [PowerManager.hpp](Core/Inc/PowerManager.hpp) | PowerManager接口定义 |

### 10.3 硬件配置文件

| 文件                            | 功能             |
| ------------------------------- | ---------------- |
| `RM2024-SuperCap-F3-V0.2.ioc` | STM32CubeMX 项目 |
| `adc.c/h`                     | ADC配置          |
| `hrtim.c/h`                   | HRTIM配置        |
| `can.c/h`                     | CAN配置          |

---

## 11. 开发注意事项

### 11.1 实时性要求

- **高速循环**: ~50μs (20kHz)，禁止使用 `HAL_Delay`
- **低速循环**: 1ms (1kHz)，可以使用短延时
- **中断优先级**: HRTIM > TIM2 > CAN

### 11.2 内存使用

- **Flash限制**: 64KB，当前使用约 37%
- **RAM限制**: 12KB，当前使用约 25%
- **ADC缓冲区**: 使用 `.BUFFER` section，避免占用常规RAM

### 11.3 代码风格

- C++ 代码使用命名空间封装
- 使用 HAL 库调用外设
- 关键数据使用 `static` 或 `namespace` 封装
- 中断函数使用 `extern "C"` 声明

---

## 12. 扩展开发

### 12.1 移植到其他MCU

本项目可移植到其他STM32 MCU（如 STM32G474）：

1. **必需外设**:

   - HRTIM 或高频定时器
   - 双ADC + DMA
   - CAN 或 UART
2. **需要修改的文件**:

   - `.ioc` 文件（CubeMX重新生成）
   - `hrtim.c/h` 或对应定时器配置
   - `adc.c/h` 配置参数
   - `calibration.hpp` 校准参数
3. **用户代码**:

   - 所有 `Core/` 中的 `.cpp/.hpp` 文件可直接复用

### 12.2 添加新功能

**添加新的保护逻辑**:

1. 在 `ErrorChecker` 命名空间添加检测函数
2. 定义新的错误代码（如 `ERROR_XXX`）
3. 在 `handleErrorState()` 中调用
4. 更新蜂鸣器提示

**修改PID参数**:

1. 修改 [PowerManager.cpp:168-181](Core/Src/PowerManager.cpp:168-181) 的PID构造参数
2. 使用 Ozone 实时采样调试
3. 可使用 `Test::testICycle()` 测试电流阶跃响应

---

## 13. 参考资料

- **STM32F334 数据手册**: [链接](https://www.st.com/resource/en/datasheet/stm32f334r8.pdf)
- **STM32CubeProgrammer 手册**: [链接](https://www.st.com/resource/en/user_manual/um2237-stm32cubeprogrammer-software-description-stmicroelectronics.pdf)
- **项目 README**: [readme.md](readme.md)
- **编译烧录指南**: [BUILD_AND_FLASH_GUIDE.md](BUILD_AND_FLASH_GUIDE.md)

---

## 附录：快速命令参考

```bash
# 编译
make -j HARDWARE_ID=101

# 清理
make clean

# 烧录 (STM32CubeProgrammer)
STM32_Programmer_CLI.exe -c port=SWD -e all -d build/RM2024-SuperCap-F3-V1R.hex -v -hardRst

# 查看内存使用
make size
```

---

**文档版本**: v1.0
**最后更新**: 2025-11-03
**维护者**: Claude AI Assistant

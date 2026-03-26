# 超级电容控制系统

## 🎯 主要功能
这是一个超级电容充放电控制系统，实现了智能功率管理、多重保护机制和CAN通信功能。

## 📊 系统架构

### 核心主函数
```c
void A_Timing_Ranking_Idea(void){
    TEST_OUT_HIGH;
    TimingCNT++;

    ADC_Convert_To_Reality();

    State_Change();
    Power_Loop();

    ADC_Value_AVG_Compute(DIV512); // 运行频率：100K/512 = 190hz

    if ((TimingCNT & DIV512) == 1) Power_Calculations();        // 运行频率：100K/128 = 190hz
    if ((TimingCNT & DIV512) == 2) Can_SendMess(&sCAN_TX_data); // 运行频率：100K/128 = 190hz
    // 计算平均值、功率计算、CAN发送。数据更新将以最慢的那个为准。

    if ((TimingCNT & DIV1024) == 10) CAN_ReceiveDataRefresh_Flag = 1; //刷新频率：100K/1024 = 97hz
    if ((TimingCNT & DIV8192) == 3) Temperature_Calculations(); // 运行频率：100K/8192 = 12hz
    if ((TimingCNT & DIV8192) == 10) LED_Refresh();             // 运行频率：100K/8192 = 12hz
    if ((TimingCNT & DIV8192) == 10) HAL_IWDG_Refresh(&hiwdg);  // 运行频率：100K/8192 = 12hz

    TEST_OUT_LOW;
}
```

> 💡 **建议**: 看代码时可以复制到新窗口，`A_Timing_Ranking_Idea(void)` 是整个系统的总脉络

## 🔧 关键函数说明

| 函数名 | 功能描述 | 运行频率 |
|--------|----------|----------|
| `State_Change()` | 状态检测与切换 | 100KHz |
| `Power_Loop()` | 根据状态执行功率控制 | 100KHz |
| `ADC_Value_AVG_Compute()` | 计算ADC平均值供操作手查看 | 190Hz |
| `Power_Calculations()` | 功率计算并显示给操作手 | 190Hz |
| `Can_SendMess()` | CAN数据发送（已配置） | 190Hz |
| `Temperature_Calculations()` | 温度计算 | 12Hz |
| `LED_Refresh()` | LED状态刷新 | 12Hz |

### 🛠️ 初始化函数

- **`ADC_Curve_Fitting()`** - UID准备函数
- **`FDCAN_Filter_Init()`** - CAN通讯初始化
- **`Power_Loop_Parameter_Init()`** - 充放电参数初始化

## 📋 系统状态定义

### 超级电容状态枚举
```c
typedef enum
{
  DISCHARGE = 0,              // 放电
  CHARGE = 1,                 // 充电
  WAIT = 2,                   // 等待
  SOFRSTART_PROTECTION = 3,   // 软启动保护
  OCP_PROTECTION = 4,         // 过流保护
  OVP_BAT_PROTECTION = 5,     // 电池过压保护
  UVP_BAT_PROTECTION = 6,     // 电池欠压保护
  UVP_CAP_PROTECTION = 7,     // 电容欠压保护
  OTP_PROTECTION = 8          // 过温保护
} SuperCapStateTypeDef;
```

### 超级电容就绪状态
```c
typedef enum
{
  UNREADY = 0,    // 未就绪
  READY = 1,      // 就绪
} SuperCapReadyTypeDef;
```

## 🏷️ 数据结构

### 状态标志位
```c
typedef struct{
  uint8_t SoftStart_bit;     // 超级电容软起动
  uint8_t Charge_bit;        // 超级电容充电(1充电，0放电)
  uint8_t Enable_bit;        // 超级电容使能(1使能，0失能)
  
  uint8_t CAN_Offline_bit;   // 控制板CAN离线检测(1离线，0在线)
  uint8_t UVP_Bat_bit;       // 电池欠压保护(1欠压，0正常)
  uint8_t UVP_Cap_bit;       // 电容组过放保护(1过放，0正常)
  uint8_t OTP_MOS_bit;       // 控制板温度保护(1过温，0正常)
  uint8_t OTP_CAP_bit;       // 电容组过温保护(1过温，0正常)
  uint8_t OCP_bit;           // 控制板过流保护(1过流，0正常)
  uint8_t OVP_Cap_bit;       // 电容组过压保护(1过压，0正常)
  uint8_t OVP_Bat_bit;       // 控制板过压保护(1过压，0正常)
} StateFlagsTypeDef;
```

> 📝 **注释**: 本来计划使用位域定义，但调试时发现位域变量不显示变量名，因此使用uint8类型

### CAN通信数据结构

#### 📥 接收数据（电控发送）
```c
typedef struct {
  uint8_t Charge;                    // 充电控制(1充电，0放电)
  uint8_t Enable;                    // 使能控制(1使能，0失能)
  uint8_t PowerLimit;                // 裁判系统功率限制
  uint8_t ChargePower;               // 超级电容充电功率
  uint8_t Dead;                      // 机器人死亡状态(未使用)
  float PowerLimitAfterOffset;       // 偏移后功率限制
  float ChargePowerAfterOffset;      // 偏移后充电功率
} CAN_ReceiveDataTypeDef;
```

#### 📤 发送数据（发送给电控）
```c
typedef struct {
  uint8_t SuperCapEnergy;            // 超级电容可用能量(0-100%)
  uint8_t ChassisPower;              // 底盘实时功率(0-255W)
  SuperCapReadyTypeDef SuperCapReady; // 超级电容可用标志
  SuperCapStateTypeDef SuperCapState; // 超级电容状态标志
  uint8_t VoltageBat;                // 电池电压*10
  uint8_t DebugOut_BatPower;         // 调试输出电池功率
  int8_t DebugOut_SuperCapPower;     // 调试输出超电功率
} CAN_TransmitDataTypeDef;
```

### ADC数据结构
```c
typedef struct {
  uint32_t Vcap;    // 电容电压
  uint32_t Vbat;    // 电池电压
  uint32_t Ibat;    // 电池电流
  uint32_t Icap;    // 电容电流
  uint32_t Tmos;    // MOS管温度
  uint32_t Tcap;    // 电容温度
} ADC_ValueTypeDef;
```

> ⚠️ **注意**: 该结构体用途多样 - 既作为累加求和使用，也用于平均值计算，还用于瞬时控制

## 🚀 核心控制流程

### 1. 系统初始化
```c
void Power_Loop_Parameter_Init(void){
    TIM1_PWM2_BREAK;           // 关闭半桥PWM，防止初始值导致意外
    PMOS_OFF;                  // 关闭PMOS
    
    // 初始状态设置
    sStateBit.SoftStart_bit = 1;    // 开机必须软启动
    sStateBit.Charge_bit = 1;       // 默认充电状态
    sStateBit.Enable_bit = 0;       // 等待CAN使能
    sStateBit.CAN_Offline_bit = 1;  // 默认CAN离线
    
    // PID参数初始化
    // Buck电压环PID
    sPID_buck_V.Kp = PID_BUCK_V_KP;
    sPID_buck_V.Ki = PID_BUCK_V_KI;
    // ... 其他PID参数
}
```

### 2. 软启动保护
```c
void Soft_Start_Loop(void){
    // 软启动期间PMOS关闭，通过寄生二极管小电流充电
    if (sCAN_RX_data.Charge && sCAN_RX_data.Enable) {
        // 电流环控制
        PID_Compute(&sPID_buck_I, Icap, SOFTSTART_CHARGE_ICAP, &PID_CurrentLoopOut);
        // 电压环控制
        PID_Compute(&sPID_buck_V, Vcap, SOFTWARE_OVP_VCAP, &PID_VoltageLoopOut);
        
        // PWM输出竞争选择最小值
        Loop_Competition_Buck(PID_CurrentLoopOut, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);
        
        // 设置PWM和ADC采样时间
        SET_PWM_COMPARE(TIM1_PWM2_Comapre);
        SET_ADC_TRIGGER_COMPARE((TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 2));
        
        SuperCap_Enable();
    }
    
    // 充电到5V以上退出软启动
    if (Vcap > 5) {
        SaftChargeOutDelay++;
        if (SaftChargeOutDelay >= COUNT_TO_1S_ON_100Khz && sCAN_RX_data.Charge == CHARGE) {
            sStateBit.SoftStart_bit = 0;
            sStateBit.UVP_Cap_bit = 1;
        }
    }
}
```

### 3. 充电控制
```c
void Charge_Loop(void){
    // 安全功率限制
    SafeChargePcap = Vcap * SAFE_CHARGE_ICAP;
    P_ChargeCap = (sCAN_RX_data.ChargePowerAfterOffset > SafeChargePcap) ? 
                  SafeChargePcap : sCAN_RX_data.ChargePowerAfterOffset;
    
    // 功率环控制
    Pcap = Vcap * Icap;
    PID_Compute(&sPID_PowerLoop_Charge, Pcap, P_ChargeCap, &PID_PowerLoopOut);
    
    // 电压环控制
    PID_Compute(&sPID_buck_V, Vcap, SOFTWARE_OVP_VCAP, &PID_VoltageLoopOut);
    
    // 双环控制竞争
    Loop_Competition_Buck(PID_PowerLoopOut, PID_VoltageLoopOut, &TIM1_PWM2_Comapre);
    
    // 断电检测
    Power_Down();  // 检测电池电流，过低时失能电容组
}
```

### 4. CAN通信管理
```c
// 接收回调
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &sFDCAN1_RxHeader, CAN_RX_Buff);
    CAN_WDG_Count = 0;  // 清零离线计数
}

// 离线处理
void CAN_Offline_Loop(void){
    SuperCap_Disable();
    // 清除所有PID积分
    
    if (CAN_WDG_Count == 1) {
        CAN_ONline_Delay++;
        // 连续收到10次充电指令才判断在线
        if (CAN_ONline_Delay >= 10 && sCAN_RX_data.Charge == CHARGE) {
            sStateBit.CAN_Offline_bit = 0;
        }
    }
}
```

## ⚡ PID控制说明

系统采用多环PID控制：
- **电压环**: 限制最大电压防止过压
- **电流环**: 限制最大电流防止过流  
- **功率环**: 控制充放电功率

> 🔧 **技巧**: 每个电压值对应特定占空比，通过 `PID->OutMax * 0.04f` 计算电压-占空比比例关系

## 📱 使用说明

1. **上电流程**: 系统自动进入软启动模式
2. **CAN通信**: 等待电控发送使能和充电指令
3. **状态监控**: 通过CAN实时上报系统状态
4. **保护机制**: 多重硬件和软件保护确保安全

## 🛡️ 安全特性

- ✅ 软启动保护防止冲击
- ✅ 过压/欠压保护
- ✅ 过流保护
- ✅ 过温保护  
- ✅ CAN通信看门狗
- ✅ 断电检测

---

> 💻 **开发建议**: 复制代码到新窗口查看，便于理解整体架构

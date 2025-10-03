# 项目说明

本项目核心代码结构如下，通过点击下方图片可以快速跳转到主要函数逻辑解析部分。

[![最最主要函数](image/23066d85f33a5f937d9f46874e647cd0.png)](#func-logic)

---

## 目录

- [主要函数逻辑](#func-logic)
- [各函数说明](#functions)
- [建议阅读方式](#suggestions)

---

## <a name="func-logic"></a>主要函数逻辑

本节介绍项目最核心的几个函数及其作用：

- `State_Change()`:  
  检测状态的变化。

- `Power_Loop()`:  
  根据检测到的状态进行相应处理。

- `ADC_Value_AVG_Compute(DIV512)`:  
  用于操作手查看板子状态的函数，带有频率设置，非每次都进行计算。

- `Power_Calculations()`:  
  计算功率并展示给操作手。

- `Can_SendMess`:  
  CAN 总线数据发送，英雄代码已经配好，无需额外配置。

---

## <a name="functions"></a>各函数说明

- **ADC_Curve_Fitting(void)**  
  UID 准备函数，用于曲线拟合。

- **FDCAN_Filter_Init**  
  CAN 通讯相关的初始化函数。

- **Power_Loop_Parameter_Init(void)**  
  充放电相关参数初始化函数。

---

## <a name="suggestions"></a>建议

- **阅读建议**：  
  建议将代码复制到新窗口，重点关注 `A_Timing_Ranking_Idea(void)` 这个函数，它是整个项目的总脉络。
- 详细代码逻辑请结合本 README 对照源文件进行分析。

---

如有疑问或建议，欢迎 issue 或讨论交流！


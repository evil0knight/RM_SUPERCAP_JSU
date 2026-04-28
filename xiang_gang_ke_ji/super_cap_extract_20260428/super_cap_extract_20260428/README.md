# 超电相关代码提取说明

本目录按原相对路径复制了当前仓库里和超电链路直接相关的代码，方便单独查看、打包或继续发给别人。

## 先看结论

1. 超电板本体挂在 `chassis` 工程的 `CAN2`。
2. 超电板协议固定为：
   - 主控发给超电板：`0x061`
   - 超电板回给主控：`0x051`
3. 底盘和云台之间用于传 `cap_mode` 的双板通信走 `CAN1`：
   - `gimbal -> chassis`：`0x012 -> 0x011`
   - `chassis -> gimbal`：`0x011 -> 0x012`
4. 当前代码里“持续使能超电”的实际链路不是单点常量开关，而是 `5ms` 周期任务每拍重新判定并重发命令。
5. 一个很重要的现状：`cap_mode` 现在基本只在 `gimbal` 侧赋值并通过双板通信发给 `chassis`，但 `chassis` 当前超电策略代码已经不再真正读取它。

## 持续使能链路

实际周期链路是：

`chassis/application/robot_task.h`
-> `RobotTask()`
-> `ChassisTask()`
-> `LimitChassisOutput()`
-> `UpdateSuperCapOutputState()`
-> `SuperCapEnable()/SuperCapDisable()`
-> `SuperCapSend()`

也就是说，超电使能位不是只在初始化时写一次，而是底盘任务每 `5ms` 都会根据当前状态重新决定并发送。

## 超电命令和回包格式

定义在：

- `chassis/modules/super_cap/super_cap.h`

发送给超电板的关键字段：

- `bit0`: `enableDCDC`
- `bit1`: `systemRestart`
- `bytes1-2`: `refereePowerLimit`
- `bytes3-4`: `refereeEnergyBuffer`

从超电板读回的关键字段：

- `byte0`: `errorCode`
- `bytes1-4`: `chassisPower` (`float`)
- `bytes5-6`: `chassisPowerLimit`
- `byte7`: `capEnergyPercent`

## 当前真正决定超电开关的条件

核心逻辑在：

- `chassis/application/chassis/chassis_motion.c`
- `chassis/application/chassis/chassis_spin.c`
- `chassis/application/chassis/chassis.c`

当前真正参与判定的条件主要有：

- 超电是否在线
- 裁判系统是否允许底盘输出
- 底盘是否处于 `CHASSIS_ZERO_FORCE`
- 超电板是否存在 `bit0-bit6` 的真实硬错误
- 超电板是否回报 `bit7` 输出禁用
- 当前是否已经形成有效的超电辅助预算 `assist_applied_w`
- 超电电量百分比和裁判缓冲能量是否过门槛

## 你很可能会关心的一个点

`gimbal/application/cmd/robot_cmd.c` 里在底盘非零力时会默认：

- `chassis_cmd_send.cap_mode = SUPER_CAP_ON`

并且该字段也确实存在于：

- `gimbal/application/robot_def.h`
- `chassis/application/robot_def.h`

但我核对了当前 `chassis` 实际运行代码，超电策略路径里已经搜不到对 `cap_mode` 的实际消费。  
也就是说，现在超电是否工作，主要看底盘本地状态机，不再直接看上层 `cap_mode`。

## 故障/离线恢复逻辑

关键代码在：

- `chassis/modules/super_cap/super_cap.c`
- `chassis/application/chassis/chassis_motion.c`

当前行为：

- 约 `200ms` 没收到 `0x051` 回包就判超电离线
- 离线、裁判禁用输出、零力模式时立即拉低 DCDC 请求
- 存在真实硬错误时进入 `1s 关 / 1s 开` 的方波重试
- 在线且允许输出但当前没真正加 bonus 时，也会保持 `READY` 使能，不是“没辅助就关”

## UI/显示相关

如果你还想看超电状态怎么显示到客户端，重点在：

- `chassis/application/chassis/chassis_ui.c`
- `chassis/modules/referee/rm_referee.h`

这里只保留了状态聚合和枚举定义，没有额外复制 `referee_task.c` 那整套绘图实现，因为它更多是显示层，不是超电控制本体。

## 本目录包含的文件分组

### 1. 超电本体协议和收发

- `chassis/modules/super_cap/super_cap.h`
- `chassis/modules/super_cap/super_cap.c`

### 2. 底盘侧超电策略、功率叠加、DCDC 状态机、UI 聚合

- `chassis/application/chassis/chassis.c`
- `chassis/application/chassis/chassis_motion.c`
- `chassis/application/chassis/chassis_spin.c`
- `chassis/application/chassis/chassis_private.h`
- `chassis/application/chassis/chassis_ui.c`
- `chassis/application/robot_def.h`
- `chassis/modules/referee/rm_referee.h`

### 3. 底盘侧任务周期与入口

- `chassis/application/robot.c`
- `chassis/application/robot_task.h`

### 4. 底盘侧 CAN 底层与双板通信

- `chassis/Src/can.c`
- `chassis/bsp/can/bsp_can.h`
- `chassis/bsp/can/bsp_can.c`
- `chassis/modules/can_comm/can_comm.h`
- `chassis/modules/can_comm/can_comm.c`

### 5. 云台侧对超电的上层控制意图与双板发送

- `gimbal/application/robot_def.h`
- `gimbal/application/cmd/robot_cmd.c`
- `gimbal/application/cmd/robot_cmd_private.h`
- `gimbal/Src/can.c`
- `gimbal/bsp/can/bsp_can.h`
- `gimbal/bsp/can/bsp_can.c`
- `gimbal/modules/can_comm/can_comm.h`
- `gimbal/modules/can_comm/can_comm.c`

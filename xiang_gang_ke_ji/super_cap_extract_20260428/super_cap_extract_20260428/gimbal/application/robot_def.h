/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// #define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
#define GIMBAL_BOARD // 云台板

// ==========================================
// [新增] 上岛辅助机构开关 (取消注释以启用上岛)
// ==========================================
// #define USE_ISLAND_ACTION // [条件编译] 启用上岛辅助机构（前履带+后抬升），注释此行则禁用上岛

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_DEG 0.0f // 将 yaw 对正基准统一成 DM 硬件零点 0 度，目的是cmd 初始化与 offset 计算都已围绕 0 度闭环，继续保留旧机械角只会让双板调试语义分叉。
#define PITCH_HORIZON_ECD 0 // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 41 // 将云台上抬软件限位收紧到 25 度，目的是用户反馈 45 度抬头过高，降低上限同时保留足够仰角空间。
#define PITCH_MIN_ANGLE -10 // 云台陀螺仪竖直方向最小角度
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 80 // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 16.0f // 英雄需要修改为3508的15.7f
#define NUM_PER_CIRCLE 9 // 拨盘一圈的装载量
// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 450 // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 415 // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0 // 云台旋转中心距底盘几何中心的距离,前后方向,云台位于正中心时默认设为0
#define CENTER_GIMBAL_OFFSET_Y 0 // 云台旋转中心距底盘几何中心的距离,左右方向,云台位于正中心时默认设为0
#define RADIUS_WHEEL 60 // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换
// 定义前轮履带默认速度目标值，目的是用户需要直接改一个明确配置项就能调整履带速度，而不是去状态机里找硬编码常量。
#define FRONT_TRACK_SPEED_REF_DEFAULT 10.0f

// ==========================================
// [新增] 机器人物理模型参数 (用于功率控制和坡道力矩补偿)
// ==========================================
#define ROBOT_MASS 22.4f // 机器人质量 (kg) [需根据实际称重修改]
#define ROBOT_COG_H 0.31f // 重心高度 (m) [需根据CAD或实测修改]
#define GRAVITY_ACC 9.8f // 重力加速度 (m/s^2)

// 几何参数单位转换 (mm -> m), 供力学解算使用
#define RADIUS_WHEEL_M (RADIUS_WHEEL / 1000.0f)
#define WHEEL_BASE_M (WHEEL_BASE / 1000.0f)
#define TRACK_WIDTH_M (TRACK_WIDTH / 1000.0f)
#define HALF_WHEEL_BASE_M (WHEEL_BASE_M / 2.0f)
#define HALF_TRACK_WIDTH_M (TRACK_WIDTH_M / 2.0f)

// 电机扭矩系数转换
// M3508/M2006: 电流Raw(0-16384) -> 扭矩(Nm) 的系数约为 0.0003662
// 补偿算法需要反向: 目标扭矩(Nm) -> 目标电流Raw
#define TORQUE_2_CURRENT_COEF (1.0f / 0.0003662109375f)
// ==========================================
// [新增] IMU 轴向映射与符号修正
// 用户反馈: 真实的 Pitch 轴对应陀螺仪的 Roll 数据
// ==========================================
#define GIMBAL_PITCH_AXIS Pitch // [轴互换后] 直接映射 (EKF 层已交换)
#define GIMBAL_ROLL_AXIS Roll // [轴互换后] 直接映射
#define GIMBAL_YAW_AXIS Yaw // Yaw 轴保持不变

#define GIMBAL_PITCH_SIGN (-1.0f) // Pitch 轴方向修正 (根据 GYRO2GIMBAL_DIR_ROLL 原始定义为 1, 此处可按需调整)
#define GIMBAL_ROLL_SIGN (1.0f) // Roll 轴方向修正
#define GIMBAL_YAW_SIGN (1.0f) // Yaw 轴方向修正

#define GYRO2GIMBAL_DIR_ROLL 1 // 陀螺仪 data相较于云台的roll的方向,1为相同,-1为相反

// ==========================================
// [新增] 底盘速度限制参数 (m/s)
// 用于遥控器控制时正确映射摇杆值到底盘速度
// ==========================================
#define MAX_CHASSIS_VX_SPEED 6.0f // 最大前后速度 (m/s)
#define MAX_CHASSIS_VY_SPEED 6.0f // 最大左右平移速度 (m/s)

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 弹速枚举是机器人控制与发射模块的公共类型，目的是让 `robot_def.h`
// 自己完整声明本工程真正需要的通用数据类型，避免把无关模块的类型依赖继续带进公共配置头。
typedef enum {
    BULLET_SPEED_NONE = 0,
    BIG_AMU_12 = 12,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_20 = 20,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

// 应用状态
typedef enum {
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum {
    CHASSIS_ZERO_FORCE = 0, // 电流零输入
    CHASSIS_ROTATE, // 小陀螺模式
    CHASSIS_NO_FOLLOW, // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE, // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE, // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
    GIMBAL_CALI_MODE,
} gimbal_mode_e;

// 发射模式设置
typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;

// 超级电容模式设置
typedef enum {
    SUPER_CAP_OFF = 0,
    SUPER_CAP_ON,
} super_cap_mode_e;
typedef enum {
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON, // 摩擦轮开启
} friction_mode_e;

#ifdef USE_ISLAND_ACTION // [条件编译] 仅在启用上岛机构时编译前履带和抬升枚举
typedef enum {
    FRONT_TRACK_OFF = 0, // 前履带关闭
    FRONT_TRACK_ON, // 前履带开启
} front_track_mode_e;

typedef enum {
    LIFT_OFF = 0, // 抬升关闭
    LIFT_HOLD, // 抬升保持当前位置
    LIFT_ADJUST, // 抬升根据拨轮调整目标高度
    LIFT_AUTO_LEVEL, // 基于底盘pitch陀螺仪闭环自动调平，目的是上坡后松开拨轮即可自动抬升后端保持底盘水平
    LIFT_RETRACT, // 快速收腿模式，目的是左拨杆回中时全速反向收腿，撞限位后自动停止
} lift_mode_e;
#endif // USE_ISLAND_ACTION

typedef enum {
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE, // 弹舱盖关闭
} lid_mode_e;

typedef enum {
    LOAD_STOP = 0, // 停止发射
    LOAD_REVERSE, // 反转
    LOAD_1_BULLET, // 单发
    LOAD_2_BULLET, // 两连发（英雄机器人专用）
    LOAD_3_BULLET, // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx; // 前进方向速度
    float vy; // 横移方向速度
    float wz; // 旋转速度
    float offset_angle; // 真实云台相对底盘的物理夹角，目的是底盘平移坐标变换和 UI 显示都必须基于真实相对姿态，不能被“虚拟前方”语义污染。
    float follow_offset_angle; // 底盘跟随闭环使用的虚拟前方误差，目的是V 键掉头后需要让“新的云台朝向”被视为前方，但同时又不能改掉真实夹角给平移坐标变换的语义。
    float gimbal_gyro_z; // 来自云台的陀螺仪z轴角速度前馈
    float gimbal_cmd_wz; // 来自云台的底盘旋转控制量
    uint8_t calibrate_imu; // 请求底盘IMU校准
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
    super_cap_mode_e cap_mode;
    float gimbal_pitch_deg; // 把云台实际pitch角随双板命令发送到底盘，目的是底盘板 UI 需要基于机构真实位置驱动俯仰滑块。
    uint8_t friction_on; // 把摩擦轮实际启停状态随双板命令发送到底盘，目的是底盘板 UI 的 fric 指示必须和上板发射使能保持一致。
    Bullet_Speed_e ui_bullet_speed; // 把当前预选弹速随双板命令发送到底盘，目的是裁判 UI 要在 F 旁直接显示 12/16 档位，而不是再去猜测瞬时发射命令。
    uint8_t ui_refresh_request; // 发送一次性 UI 刷新请求到底盘板，目的是手动重绘必须复用现有双板命令链，而不是额外开一条旁路。
    uint8_t follow_transition_request; // 显式标记“小陀螺退跟随”的接管边沿，目的是双板链路是覆盖式最新帧语义，只靠模式值本身无法稳定表达边沿事件。
    uint8_t follow_brake_request; // 显式请求底盘在跟随模式下临时只做阻尼刹停，目的是V 键掉头期间需要让底盘平滑停住而不是继续追随云台角速度与历史跟随误差。
#ifdef USE_ISLAND_ACTION // [条件编译] 仅在启用上岛机构时包含辅助机构控制字段
    front_track_mode_e front_track_mode; // 独立描述前履带开关状态，目的是避免复用底盘主模式后让跟随与上岛辅助机构互相覆盖
    lift_mode_e lift_mode; // 独立描述抬升状态机，目的是让抬升保持与调高语义在双板两侧都保持一致
    float front_track_speed_ref; // 下发前履带速度参考，目的是把履带调速接口预留在双板协议里，底盘侧不再硬编码启停外的行为
    float lift_dial_input; // 下发抬升拨轮归一化输入，目的是底盘侧基于真实电机反馈积分目标高度，避免云台板盲算位置
#endif // USE_ISLAND_ACTION
    // UI部分
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd 发布的云台控制数据，由 gimbal 订阅。
typedef struct
{ // 这里刻意只保留云台目标角与模式，目的是把双板协议收口成“目标是什么”而不是“底层 PID 该怎么清”，避免上层消息继续渗透到底层运行时细节。
    float yaw; // 下发给 gimbal 的 yaw 累计角目标，目的是上层只表达云台最终要转到哪里，不再通过协议夹带任何 PID 内部状态操作。
    float pitch; // 下发给 gimbal 的 pitch 目标角，目的是失能恢复和模式切换都统一通过目标同步完成，不再依赖额外的 reset 标志位。
    gimbal_mode_e gimbal_mode; // 下发给 gimbal 的工作模式，目的是底层仍按零力、陀螺仪和自由模式做主状态分发，但模式之外不再附带控制器内部清理语义。
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;

    uint8_t rest_heat; // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
} Chassis_Upload_Data_s;

typedef struct
{
    attitude_t gimbal_imu_data;
    float yaw_motor_single_round_angle;
    uint8_t yaw_motor_online; // 回传 yaw 电机在线状态，目的是cmd 侧要在掉线和复活边沿冻结 offset 并重同步目标，不能继续盲信机械角。
    uint8_t pitch_motor_online; // 回传 pitch 电机在线状态，目的是cmd 侧需要在掉线或零力恢复后把目标重新贴到当前姿态并清掉残留，而不是继续沿用旧锁存目标。
} Gimbal_Upload_Data_s;

typedef struct
{
    uint8_t bullet_fired_flag; // 发射确认标志 (1=确认发射, 0=未确认)
    uint8_t empty_flag; // 缺弹标志 (1=缺弹, 0=正常)
    uint16_t fire_count; // 累计确认发射计数
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H


#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"
#include "daemon.h"
#include "robot_def.h"

#define SUPERCAP_CAN_TX_ID 0x061 // F407发送到超电板的ID
#define SUPERCAP_CAN_RX_ID 0x051 // 从超电板接收的ID

/* 错误代码定义 */
#define ERROR_UNDER_VOLTAGE 0x01 // Bit 0: 欠压
#define ERROR_OVER_VOLTAGE 0x02 // Bit 1: 过压
#define ERROR_BUCK_BOOST 0x04 // Bit 2: Buck-Boost转换器错误
#define ERROR_SHORT_CIRCUIT 0x08 // Bit 3: 短路
#define ERROR_HIGH_TEMPERATURE 0x10 // Bit 4: 高温
#define ERROR_NO_POWER_INPUT 0x20 // Bit 5: 无电源输入
#define ERROR_CAPACITOR 0x40 // Bit 6: 电容故障
#define ERROR_OUTPUT_DISABLED 0x80 // Bit 7: 输出禁用状态

/* 功率和能量限制 */
#define POWER_LIMIT_MIN 30 // 最小功率限制 (W)
#define POWER_LIMIT_MAX 250 // 当前协议里这个字段表示真实裁判功率限制，目的是保持在超电板认可的 30~250W 范围内，避免因为把本地总预算塞进去而触发输出关断。
#define ENERGY_BUFFER_MAX 300 // 最大能量缓冲 (J)

#pragma pack(1)

/* 发送到超电板的数据结构 (ID: 0x061) */
typedef struct
{
    uint8_t enableDCDC : 1; // Bit 0: DCDC使能标志 (1=使能, 0=禁用)
    uint8_t systemRestart : 1; // Bit 1: 系统重启命令 (1=重启)
    uint8_t resv0 : 6; // Bit 2-7: 保留位
    uint16_t refereePowerLimit; // 裁判系统功率限制值 (单位: W, 范围: 30-250W)
    uint16_t refereeEnergyBuffer; // 裁判系统能量缓冲值 (单位: J, 范围: 0-300J)
    uint8_t resv1[3]; // 3字节保留位
} SuperCap_Tx_Msg_s;

/* 从超电板接收的数据结构 (ID: 0x051) */
typedef struct
{
    uint8_t errorCode; // 错误代码 + 输出状态 (bit7: 输出禁用, bit0-6: 错误标志)
    float chassisPower; // 底盘实时功率 (单位: W, IEEE 754浮点数)
    uint16_t chassisPowerLimit; // 底盘功率限制值 (单位: W)
    uint8_t capEnergyPercent; // 电容能量百分比 (0-255对应0-100%)
} SuperCap_Rx_Msg_s;

#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins; // CAN实例
    SuperCap_Tx_Msg_s tx_msg; // 发送消息
    SuperCap_Rx_Msg_s rx_msg; // 接收消息
    DaemonInstance *daemon_ins; // 看门狗实例
    uint8_t is_online; // 在线状态标志
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容
 *
 * @param supercap_config 超级电容初始化配置
 * @return SuperCapInstance* 超级电容实例指针
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config);

/**
 * @brief 发送超级电容控制信息
 *
 * @param instance 超级电容实例
 * @param data 超级电容控制信息
 */
void SuperCapSend(SuperCapInstance *instance);

/**
 * @brief 设置超级电容功率限制
 *
 * @param cap 超级电容实例
 * @param power_limit 功率限制 裁判系统
 */
void SuperCapSetPowerLimit(SuperCapInstance *instance, uint16_t power_limit);

/**
 * @brief 设置裁判系统能量缓冲
 * @param instance 超级电容实例
 * @param energy_buffer 能量缓冲值 (0-300J)
 */
void SuperCapSetEnergyBuffer(SuperCapInstance *instance, uint16_t energy_buffer);

/**
 * @brief 发送系统重启命令
 * @param instance 超级电容实例
 */
void SuperCapSystemRestart(SuperCapInstance *instance);
/**
 * @brief 获取底盘实时功率
 * @param instance 超级电容实例
 * @return float 底盘功率 (W)
 */
float SuperCapGetChassisPower(SuperCapInstance *instance);

/**
 * @brief 获取电容能量百分比
 * @param instance 超级电容实例
 * @return float 能量百分比 (0.0-100.0%)
 */
float SuperCapGetEnergyPercent(SuperCapInstance *instance);

/**
 * @brief 获取超电板回传的底盘功率上限
 * @param instance 超级电容实例
 * @return uint16_t 超电板当前认为可提供的底盘功率上限
 */
uint16_t SuperCapGetReportedPowerLimit(SuperCapInstance *instance);

/**
 * @brief 获取错误代码
 * @param instance 超级电容实例
 * @return uint8_t 错误代码
 */
uint8_t SuperCapGetErrorCode(SuperCapInstance *instance);

/**
 * @brief 判断超电板是否存在真实硬错误
 * @param instance 超级电容实例
 * @return uint8_t 1=存在 bit0-bit6 错误, 0=无真实错误
 */
uint8_t SuperCapHasHardFault(SuperCapInstance *instance);

/**
 * @brief 检查超电板是否在线
 * @param instance 超级电容实例
 * @return uint8_t 1=在线, 0=离线
 */
uint8_t SuperCapIsOnline(SuperCapInstance *instance);

/**
 * @brief 检查输出是否被禁用
 * @param instance 超级电容实例
 * @return uint8_t 1=禁用, 0=使能
 */
uint8_t SuperCapIsOutputDisabled(SuperCapInstance *instance);
/**
 * @brief 使能超级电容
 *
 * @param cap 超级电容实例
 */
void SuperCapEnable(SuperCapInstance *instance);

/**
 * @brief 失能超级电容
 *
 * @param cap 超级电容实例
 */
void SuperCapDisable(SuperCapInstance *instance);

#endif // !SUPER_CAP_Hd

/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_log.h"
static SuperCapInstance *g_super_cap_instance = NULL;
static DaemonInstance *super_cap_daemon_instance;

static void SuperCapLostCallback(void *cap_ptr)
{
    SuperCapInstance *cap = (SuperCapInstance *)cap_ptr;
    LOGWARNING("[SuperCap] Connection Lost!");
    cap->is_online = 0;
    memset(&cap->rx_msg, 0, sizeof(SuperCap_Rx_Msg_s));
}

static void SuperCapRxCallback(CANInstance *can_instance)
{
    if (g_super_cap_instance == NULL)
        return;

    // 重载看门狗
    DaemonReload(g_super_cap_instance->daemon_ins);
    g_super_cap_instance->is_online = 1;

    uint8_t *rx_buff = can_instance->rx_buff;
    SuperCap_Rx_Msg_s *rx_msg = &g_super_cap_instance->rx_msg;

    // 解析接收数据 (小端序)
    // Byte 0: errorCode
    rx_msg->errorCode = rx_buff[0];

    // Byte 1-4: chassisPower (IEEE 754 float, 小端序)
    memcpy(&rx_msg->chassisPower, &rx_buff[1], 4);

    // Byte 5-6: chassisPowerLimit (uint16_t, 小端序)
    rx_msg->chassisPowerLimit = (uint16_t)(rx_buff[5] | (rx_buff[6] << 8));

    // Byte 7: capEnergyPercent (uint8_t)
    rx_msg->capEnergyPercent = rx_buff[7];

    // 调试输出
    // LOGINFO("[SuperCap] Power=%.2fW, Cap=%d%%, Error=0x%02X",
    //         rx_msg->chassisPower,
    //         (int)(rx_msg->capEnergyPercent * 100.0f / 255.0f),
    //         rx_msg->errorCode);
}
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config)
{
    // 分配内存
    g_super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(g_super_cap_instance, 0, sizeof(SuperCapInstance));

    // 配置CAN
    config->can_config.tx_id = SUPERCAP_CAN_TX_ID;
    config->can_config.rx_id = SUPERCAP_CAN_RX_ID;
    config->can_config.can_module_callback = SuperCapRxCallback;
    g_super_cap_instance->can_ins = CANRegister(&config->can_config);

    // 底盘任务以 5ms 周期持续给超电板发命令，超电板正常回包时 DaemonTask 会以 100Hz 被周期性喂狗，
    // 这里把离线阈值设成 20 拍，相当于约 200ms 内没有收到 0x051 回包就判离线，既能比旧配置更快回退，又不会被偶发单帧抖动误伤。
    Daemon_Init_Config_s daemon_config = {
        .callback = SuperCapLostCallback,
        .owner_id = g_super_cap_instance,
        .reload_count = 20,
    };
    g_super_cap_instance->daemon_ins = DaemonRegister(&daemon_config);

    // 默认配置
    g_super_cap_instance->tx_msg.enableDCDC = 0; // 默认禁用
    g_super_cap_instance->tx_msg.systemRestart = 0;
    g_super_cap_instance->tx_msg.refereePowerLimit = 80; // 默认80W
    g_super_cap_instance->tx_msg.refereeEnergyBuffer = 60; // 默认60J

    LOGINFO("[SuperCap] Initialized (TX_ID=0x%03X, RX_ID=0x%03X)",
            SUPERCAP_CAN_TX_ID, SUPERCAP_CAN_RX_ID);

    return g_super_cap_instance;
}
void SuperCapSend(SuperCapInstance *instance)
{
    if (instance == NULL || instance->can_ins == NULL)
        return;

    SuperCap_Tx_Msg_s *tx_msg = &instance->tx_msg;
    uint8_t *tx_buff = instance->can_ins->tx_buff;

    // 封装发送数据 (小端序)
    // Byte 0: [enableDCDC][systemRestart][保留×6]
    tx_buff[0] = (tx_msg->enableDCDC & 0x01) |
                 ((tx_msg->systemRestart & 0x01) << 1);

    // Byte 1-2: refereePowerLimit (uint16_t, 小端序)
    tx_buff[1] = (uint8_t)(tx_msg->refereePowerLimit & 0xFF);
    tx_buff[2] = (uint8_t)((tx_msg->refereePowerLimit >> 8) & 0xFF);

    // Byte 3-4: refereeEnergyBuffer (uint16_t, 小端序)
    tx_buff[3] = (uint8_t)(tx_msg->refereeEnergyBuffer & 0xFF);
    tx_buff[4] = (uint8_t)((tx_msg->refereeEnergyBuffer >> 8) & 0xFF);

    // Byte 5-7: 保留位
    tx_buff[5] = 0;
    tx_buff[6] = 0;
    tx_buff[7] = 0;

    // 超电命令由 200Hz 的底盘任务发出，若 CAN 邮箱长时间占满，继续在这里阻塞会直接把主控制拍拖慢，
    // 因此只给 1ms 的等待窗口，发不出去就让本拍跳过，下一拍继续覆盖式发送最新命令。
    CANTransmit(instance->can_ins, 1.0f);

    // 重启命令只发送一次，发送后立即清除
    if (tx_msg->systemRestart) {
        tx_msg->systemRestart = 0;
    }
}

void SuperCapEnable(SuperCapInstance *instance)
{
    if (instance == NULL)
        return;
    instance->tx_msg.enableDCDC = 1;
    LOGINFO("[SuperCap] DCDC Enabled");
}

void SuperCapDisable(SuperCapInstance *instance)
{
    if (instance == NULL)
        return;
    instance->tx_msg.enableDCDC = 0;
    LOGINFO("[SuperCap] DCDC Disabled");
}
void SuperCapSetPowerLimit(SuperCapInstance *instance, uint16_t power_limit)
{
    if (instance == NULL)
        return;

    // 这个字段在超电板协议里就是裁判功率限制，不是底盘本地最终总预算，
    // 因此这里只按超电板认可的 30~250W 裁判范围截断，避免下位机把更高的动态总预算错误塞进来后触发超电板主动关输出。
    if (power_limit < POWER_LIMIT_MIN) {
        power_limit = POWER_LIMIT_MIN;
    } else if (power_limit > POWER_LIMIT_MAX) {
        power_limit = POWER_LIMIT_MAX;
    }

    instance->tx_msg.refereePowerLimit = power_limit;
}

void SuperCapSetEnergyBuffer(SuperCapInstance *instance, uint16_t energy_buffer)
{
    if (instance == NULL)
        return;

    // 限制范围 0-300J
    if (energy_buffer > ENERGY_BUFFER_MAX) {
        energy_buffer = ENERGY_BUFFER_MAX;
    }

    instance->tx_msg.refereeEnergyBuffer = energy_buffer;
}

void SuperCapSystemRestart(SuperCapInstance *instance)
{
    if (instance == NULL)
        return;
    instance->tx_msg.systemRestart = 1;
    LOGWARNING("[SuperCap] System Restart Command Sent!");
}

float SuperCapGetChassisPower(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0.0f;
    return instance->rx_msg.chassisPower;
}

float SuperCapGetEnergyPercent(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0.0f;
    // 转换: 0-255 -> 0.0-100.0%
    return (float)instance->rx_msg.capEnergyPercent * 100.0f / 255.0f;
}

uint16_t SuperCapGetReportedPowerLimit(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0u;
    // 直接暴露超电板回传的功率上限，目的是让底盘策略层在不改协议的前提下就能用真实电源能力约束自己的请求。
    return instance->rx_msg.chassisPowerLimit;
}

uint8_t SuperCapGetErrorCode(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0;
    return instance->rx_msg.errorCode & 0x7F; // 只返回bit0-6
}

uint8_t SuperCapHasHardFault(SuperCapInstance *instance)
{
    // 这里刻意只看 bit0-bit6，目的是把“真实故障”和“输出当前被禁用”分开，避免状态机把 bit7 也当成硬故障反复拉闸。
    return SuperCapGetErrorCode(instance) != 0u;
}

uint8_t SuperCapIsOnline(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 0;
    return instance->is_online;
}

uint8_t SuperCapIsOutputDisabled(SuperCapInstance *instance)
{
    if (instance == NULL)
        return 1;
    return (instance->rx_msg.errorCode & ERROR_OUTPUT_DISABLED) ? 1 : 0;
}

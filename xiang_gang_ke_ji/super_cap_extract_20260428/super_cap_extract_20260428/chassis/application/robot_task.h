/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "dmmotor.h"
#include "referee_task.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"

#include "bsp_log.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
// 声明裁判 UI 线程入口，目的是任务创建恢复后需要显式暴露入口给 FreeRTOS 宏展开使用。
void StartUITASK(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    // 恢复裁判 UI 线程创建，目的是现有 UI 逻辑已经迁移到真实数据驱动路径，不再是旧测试任务，必须让线程实际运行才会在选手端生效。
    osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    uiTaskHandle = osThreadCreate(osThread(uitask), NULL);
    DMMotorControlInit(); // 启动底盘侧全部DM控制任务，目的是前履带使用独立DM线程闭环，不初始化任务只会注册实例不会真正输出
    // HTMotorControlInit(); // 没有注册HT电机则不会执行
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;) {
        // 1kHz
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;

        // 修改点：移除 &，将 float 转为 int (微秒)，使用 %d 打印
        if (ins_dt > 1)
            LOGERROR("[freeRTOS] INS Task DELAY! dt = %d us", (int)(ins_dt * 1000));

        // 视觉模块已移除，INS任务不再发送视觉数据，目的是避免无效串口链路占用1kHz任务预算
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;) {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;

        // 修改点：移除 &，将 float 转为 int (微秒)，使用 %d 打印
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task DELAY! dt = %d us", (int)(motor_dt * 1000));

        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;) {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;

        // 修改点：移除 &，将 float 转为 int (微秒)，使用 %d 打印
        if (daemon_dt > 10)
            LOGERROR("[freeRTOS] Daemon Task DELAY! dt = %d us", (int)(daemon_dt * 1000));

        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    static float robot_dt;
    static float robot_start;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    // 200Hz-500Hz
    for (;;) {
        robot_start = DWT_GetTimeline_ms();
        RobotTask();
        robot_dt = DWT_GetTimeline_ms() - robot_start;

        // 修改点：移除 &，将 float 转为 int (微秒)，使用 %d 打印
        if (robot_dt > 5)
            LOGERROR("[freeRTOS] ROBOT core Task DELAY! dt = %d us", (int)(robot_dt * 1000));

        osDelay(5);
    }
}

__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    LOGINFO("[freeRTOS] UI Task Start");
    MyUIInit();
    LOGINFO("[freeRTOS] UI Init Done");
    for (;;) {
        UITask();
        // UI 线程空转周期放宽到 10ms，目的是具体发包频率由内部调度器控制到 20Hz/10Hz，任务本身不需要 1ms 忙轮询。
        osDelay(10);
    }
}

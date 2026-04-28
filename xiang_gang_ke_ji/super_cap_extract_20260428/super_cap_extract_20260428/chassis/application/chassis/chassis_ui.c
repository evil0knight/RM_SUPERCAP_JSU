#include "chassis_private.h"

/**
 * @brief 汇总一份给裁判 UI 任务使用的实时数据快照
 *
 */
void RefereeUIUpdateData(void)
{
    uint8_t chassis_output_allowed = 1u;
    uint8_t chassis_rotate_active = 0u;

    // 先取一份当前裁判系统对底盘输出的许可状态，目的是小陀螺和 cap 的显示都应该反映“这拍是否真的允许输出”，不能继续只看抽象模式或慢一拍的回包位。
    if (referee_data != NULL) {
        chassis_output_allowed = (uint8_t)(referee_data->GameRobotState.power_management_chassis_output != 0u);
    }

    // 汇总底盘板本地和双板下发的 UI 实时数据，目的是把数据采集与 UI 绘制解耦后，裁判任务只关心显示调度，避免读多处模块造成状态不一致。
    ui_data.chassis_yaw_rate_dps = Chassis_IMU_data->Gyro[Z] * RAD_2_DEGREE;
    // 先给超电显示量安全默认值，目的是即使后面超电离线或本拍直接走回退分支，UI 也不会继续沿用上一拍残留的状态字或圆灯状态。
    ui_data.cap_state = UI_CAP_STATE_OFF;
    ui_data.cap_on = 0u;

#ifdef CHASSIS_BOARD
    if (chasiss_can_comm != NULL && CANCommIsOnline(chasiss_can_comm) != 0u) {
        // 双板在线时直接采用云台板下发的真实姿态、相对偏角与摩擦轮状态，目的是这些量在双板协同控制链里已经对齐，底盘板本地无需再自行推导。
        ui_data.gimbal_pitch_deg = chassis_cmd_recv.gimbal_pitch_deg;
        ui_data.gimbal_yaw_rate_dps = chassis_cmd_recv.gimbal_gyro_z;
        ui_data.chassis_gimbal_offset_deg = theta_format(chassis_cmd_recv.offset_angle);
        ui_data.friction_on = chassis_cmd_recv.friction_on;
        // 双板在线时直接使用上板同步过来的预选弹速，目的是 F 旁的 12/16 显示必须和上板键鼠设置同拍一致，不能在底盘板本地凭状态猜测。
        ui_data.ui_bullet_speed = chassis_cmd_recv.ui_bullet_speed;
        // 只有当前仍在自旋执行态且裁判没有切掉底盘输出时才点亮小陀螺，目的是用户关掉小陀螺或被裁判断电后，on_2 必须立刻转红，不能继续借旧模式或残余状态误亮。
        chassis_rotate_active = (uint8_t)(chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE && chassis_output_allowed != 0u);
        ui_data.robot_spin_on = chassis_rotate_active;
    } else {
        // 双板离线时冻结 pitch 和相对偏角并清零其余云台相关量，目的是相对姿态保留最后一次有效值更利于排查问题，而角速度和摩擦轮状态必须立即回落避免误导操作手。
        ui_data.gimbal_yaw_rate_dps = 0.0f;
        ui_data.friction_on = 0u;
        // 双板断链后把 F 旁档位回到 12m/s 安全默认值，目的是上一拍若停在 16m/s，继续显示旧值会误导操作者判断当前键鼠设置。
        ui_data.ui_bullet_speed = BIG_AMU_12;
        // 双板断链时主动熄灭小陀螺指示，目的是on_2 必须反映当前可确认的真实模式，不能继续沿用上一帧旧状态误导选手端。
        ui_data.robot_spin_on = 0u;
    }
#else
    // 单板构型下先给云台相关 UI 量安全默认值，目的是当前相对姿态方案主要面向双板，未补全单板数据通路前不能让显示读到未定义数据。
    ui_data.gimbal_yaw_rate_dps = 0.0f;
    ui_data.chassis_gimbal_offset_deg = 0.0f;
    ui_data.friction_on = 0u;
    // 单板模式直接复用本地命令里的预选弹速，目的是即使没有双板转发链，F 旁档位也必须仍然能反映当前键鼠设置。
    ui_data.ui_bullet_speed = chassis_cmd_recv.ui_bullet_speed;
    // 单板模式下同样按本拍真实自旋执行态点亮 on_2，目的是即使没有双板链路，也不能让 UI 继续把“模式枚举”和“实际仍在自旋输出”混成一回事。
    chassis_rotate_active = (uint8_t)(chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE && chassis_output_allowed != 0u);
    ui_data.robot_spin_on = chassis_rotate_active;
#endif

#ifdef USE_SUPER_CAP
    if (cap != NULL && SuperCapIsOnline(cap) != 0u) {
        // 超电在线时优先显示其回传的真实底盘功率，目的是功率值本身就该尽量贴近真实电源链路表现，而不是退回到底盘侧估算。
        ui_data.chassis_power_w = SuperCapGetChassisPower(cap);

        // 先处理最宽泛的 OFF 场景，目的是离线之外，裁判切掉底盘输出和零力模式同样都属于“本拍不该让超电参与”的关闭态。
        if (chassis_output_allowed == 0u || chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
            ui_data.cap_state = UI_CAP_STATE_OFF;
        } else if (SuperCapHasHardFault(cap) != 0u) {
            // 真实硬错误优先显示为 FAULT，目的是把故障与普通关闭态彻底区分开，方便场上直接判断是板子异常还是策略没开。
            ui_data.cap_state = UI_CAP_STATE_FAULT;
        } else if (SuperCapIsOutputDisabled(cap) != 0u) {
            // 只有超电板明确回报 bit7 输出禁用时才显示 DISABLED，目的是把“板子在线但当前不给输出”单独做成一类可见语义。
            ui_data.cap_state = UI_CAP_STATE_DISABLED;
        } else if (super_cap_policy_state.dcdc_state == SUPER_CAP_DCDC_ASSIST &&
                   super_cap_policy_state.assist_applied_w > 0.5f) {
            // 进入 ASSIST 的条件既看状态机也看实际已加上的功率，目的是避免状态刚切过去但本拍还没真正放出额外功率时 UI 过早亮成辅助态。
            ui_data.cap_state = UI_CAP_STATE_ASSIST;
        } else {
            // 其余在线且健康的场景统一归为 READY，目的是把“板子在、能工作、但还没真正辅助中”稳定表达出来。
            ui_data.cap_state = UI_CAP_STATE_READY;
        }

        // 圆灯继续只承担粗粒度 on/off 语义，目的是保留原有一眼看开关态的习惯，而五态细分交给右侧状态字承担。
        ui_data.cap_on = (uint8_t)(ui_data.cap_state == UI_CAP_STATE_READY || ui_data.cap_state == UI_CAP_STATE_ASSIST);
        return;
    }
#endif

    // 超电离线时回退到底盘功率控制模块的本地估算值，目的是即使辅助供电链路失效，选手端仍需要持续看到一个稳定更新的功率读数。
    ui_data.chassis_power_w = PowerControlGetChassisPower();
    ui_data.cap_state = UI_CAP_STATE_OFF;
    ui_data.cap_on = 0u;
}

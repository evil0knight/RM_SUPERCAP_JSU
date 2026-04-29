#include "chassis_private.h"

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

// 下面这些中间量只在底盘运动学与功率输出链内部流转，目的是抽到独立文件后继续局部持有，能避免主任务文件再背负大量中间状态定义。
static float vt_lf; // 缓存左前轮本拍目标速度，目的是运动学解算和功率限幅共用同一份中间结果，避免重复计算。
static float vt_rf; // 缓存右前轮本拍目标速度，目的是统一在限幅后再一次性下发到电机参考，保证四轮同拍更新。
static float vt_lb; // 缓存左后轮本拍目标速度，目的是把控制任务中的“解算”和“下发”阶段解耦，方便后续单独调试功率链路。
static float vt_rb; // 缓存右后轮本拍目标速度，目的是保持四轮参考先完整生成，再统一经过同一套功率与超电逻辑。
static float real_vx = 0.0f; // 缓存融合后的真实前进速度，目的是速度估计函数需要跨拍保留输出状态做低通滤波。
static float real_vy = 0.0f; // 缓存融合后的真实横移速度，目的是横移估计同样需要跨拍连续状态，不能每次重算后立刻丢掉。
static float real_wz = 0.0f; // 缓存融合后的真实旋转速度，目的是角速度估计最后一级低通滤波依赖历史输出。

#ifdef USE_SUPER_CAP
/**
 * @brief 更新超电 DCDC 使能请求状态
 *
 * @param chassis_output_allowed 当前拍裁判系统是否允许底盘输出
 */
void UpdateSuperCapOutputState(uint8_t chassis_output_allowed)
{
    uint32_t now;
    uint32_t fault_toggle_phase_ms;
    uint8_t desired_enable = 0u;
    uint8_t force_disable = 0u;
    super_cap_dcdc_state_e last_dcdc_state;

    if (cap == NULL) {
        return;
    }

    now = HAL_GetTick();
    last_dcdc_state = super_cap_policy_state.dcdc_state;

    // 裁判切掉输出、底盘零力或超电离线时必须立即进入关闭态，目的是这些场景都属于安全优先路径，不应再等待最小切换时间。
    if (SuperCapIsOnline(cap) == 0u ||
        chassis_output_allowed == 0u ||
        chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        super_cap_policy_state.dcdc_state = SUPER_CAP_DCDC_OFF;
        super_cap_policy_state.fault_toggle_started_ms = 0u;
        desired_enable = 0u;
        force_disable = 1u;
    } else if (SuperCapHasHardFault(cap) != 0u) {
        // 真实硬错误存在时进入固定节拍的方波重试状态，目的是严格按“先关 1 秒、再开 1 秒”的节奏恢复，复现你当前实车需要的行为。
        if (last_dcdc_state != SUPER_CAP_DCDC_FAULT_HOLD) {
            // 第一次进入故障重试态时把当前时刻记成相位起点，目的是确保每次真实故障都从“关”这一半周期开始，而不是继承上次残留相位。
            super_cap_policy_state.fault_toggle_started_ms = now;
            force_disable = 1u;
        }
        super_cap_policy_state.dcdc_state = SUPER_CAP_DCDC_FAULT_HOLD;
        fault_toggle_phase_ms =
            (now - super_cap_policy_state.fault_toggle_started_ms) % (CHASSIS_SUPER_CAP_FAULT_TOGGLE_HALF_PERIOD_MS * 2u);
        if (fault_toggle_phase_ms < CHASSIS_SUPER_CAP_FAULT_TOGGLE_HALF_PERIOD_MS) {
            // 方波前半周期固定拉低 DCDC 请求，目的是先给超电板留一整秒完全退出故障态的窗口，避免刚报码就继续强推使能。
            desired_enable = 0u;
        } else {
            // 方波后半周期固定拉高 DCDC 请求，目的是在真实故障仍未消失时也按 1 秒节拍自动重试，而不是等人工再次介入。
            desired_enable = 1u;
        }
    } else if (super_cap_policy_state.assist_applied_w > 0.5f &&
               SuperCapIsOutputDisabled(cap) == 0u) {
        // 辅助预算已经真正生效且超电板未声明输出禁用时，进入 assist 状态，目的是把“在工作”与“仅待命”区分开。
        super_cap_policy_state.dcdc_state = SUPER_CAP_DCDC_ASSIST;
        super_cap_policy_state.fault_toggle_started_ms = 0u;
        desired_enable = 1u;
    } else {
        // 在线且允许输出但当前没有追加辅助预算时仍保持 READY 使能，目的是继续允许超电板维持充电与待命，而不是每次没 bonus 就关掉 DCDC。
        super_cap_policy_state.dcdc_state = SUPER_CAP_DCDC_READY;
        super_cap_policy_state.fault_toggle_started_ms = 0u;
        desired_enable = 1u;
    }

    // 只有请求状态真的变化时才执行开关动作，目的是把状态机和报文下发解耦，避免每拍都重复打日志或重写同一命令。
    if (desired_enable != super_cap_policy_state.dcdc_requested_enable) {
        if (desired_enable == 0u) {
            if (force_disable != 0u ||
                now - super_cap_policy_state.dcdc_last_switch_ms >= CHASSIS_SUPER_CAP_DCDC_MIN_SWITCH_MS) {
                // 进入安全关闭或故障关闭时优先立即拉低请求，普通回收则仍受最小保持时间保护，避免边界状态下频繁抖动。
                SuperCapDisable(cap);
                super_cap_policy_state.dcdc_requested_enable = 0u;
                super_cap_policy_state.dcdc_last_switch_ms = now;
            }
        } else if (now - super_cap_policy_state.dcdc_last_switch_ms >= CHASSIS_SUPER_CAP_DCDC_MIN_SWITCH_MS) {
            // 重新请求使能前要求至少稳定一段时间，目的是错误刚恢复、在线状态刚回来或输出许可刚恢复时不立即重启 DCDC。
            SuperCapEnable(cap);
            super_cap_policy_state.dcdc_requested_enable = 1u;
            super_cap_policy_state.dcdc_last_switch_ms = now;
        }
    }
}
#endif // USE_SUPER_CAP

/**
 * @brief 计算每个轮毂电机的输出，正运动学解算
 *
 */
void MecanumCalculate(void)
{
    // 按标准 X 型麦轮公式解算四轮参考，目的是主任务只负责确定底盘坐标系下的 vx、vy、wz，具体四轮分配单独收在运动学文件里更清晰。
    vt_lf = chassis_vx + chassis_vy + chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = chassis_vx - chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx - chassis_vy + chassis_cmd_recv.wz * RB_CENTER;
}

/**
 * @brief 兼容前麦后全向的混合底盘解算
 *
 */
void HybridCalculate(void)
{
    float rear_rot_spd;

    // 前轮仍按麦轮公式分解，目的是这个备用解算路径主要给特殊机构调试保留，继续沿用原有符号约定最安全。
    vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;

    // 后轮只承担纵向和平面旋转分量，目的是全向轮对横向指令不敏感，因此必须和麦轮方案分开处理。
    rear_rot_spd = chassis_cmd_recv.wz * HALF_TRACK_WIDTH * DEGREE_2_RAD;
    vt_lb = chassis_vy - rear_rot_spd;
    vt_rb = chassis_vy + rear_rot_spd;
}

/**
 * @brief 根据裁判系统和超电状态对底盘输出进行限制并下发参考值
 *
 */
void LimitChassisOutput(void)
{
#ifdef USE_SUPER_CAP
    if (cap != NULL) {
        uint16_t referee_buffer_j = (uint16_t)DEFAULT_TEST_BUFFER_ENERGY_J;
        float referee_limit = DEFAULT_TEST_POWER;
        uint8_t chassis_output_allowed = 1u;

        // 先同步本拍裁判缓冲和功率许可，目的是超电下发链路必须和底盘实际使用的同一拍裁判状态对齐，不能继续吃旧值。
        if (referee_data != NULL) {
            referee_buffer_j = referee_data->PowerHeatData.buffer_energy;
            referee_limit = referee_data->GameRobotState.chassis_power_limit;
            if (referee_limit < 1.0f) {
                referee_limit = DEFAULT_TEST_POWER;
            }
            chassis_output_allowed = (uint8_t)(referee_data->GameRobotState.power_management_chassis_output != 0u);
        }

        // 通过现有 helper 下发裁判缓冲能量，目的是统一复用范围限幅逻辑，避免后续超电协议调整后底盘侧还在直接写裸字段。
        SuperCapSetEnergyBuffer(cap, referee_buffer_j);
        // 这里恢复为只同步真实裁判功率限制，目的是超电板的这个协议字段本来就被当作 refereePowerLimit 使用，
        // 若把底盘本地更高的最终预算原样塞进去，超电板会因为收到大于 250W 的“裁判功率”而直接切掉输出。
        SuperCapSetPowerLimit(cap, (uint16_t)(referee_limit + 0.5f));

        // 统一由状态机决定本拍是否请求 DCDC 使能，目的是把离线、硬错误、待命和真实辅助输出这几种时序统一收口到一个地方处理。
        UpdateSuperCapOutputState(chassis_output_allowed);

        // 在同一拍内完成超电命令下发，目的是这样 UI 和底盘功率逻辑读到的超电状态都和本拍控制动作对齐。
        SuperCapSend(cap);
    }
#endif // USE_SUPER_CAP

    // 完成功率限制后再统一下发四轮目标，目的是避免出现部分车轮先看到旧预算、部分车轮看到新预算的跨拍不一致。
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据电机反馈和底盘 IMU 数据融合估算底盘速度
 *
 */
void EstimateSpeed(void)
{
    const float dt = 0.001f;
    float imu_wz_dps = Chassis_IMU_data->Gyro[Z] * RAD_2_DEGREE;
    float v_lf = motor_lf->measure.speed_aps;
    float v_rf = -motor_rf->measure.speed_aps;
    float v_lb = motor_lb->measure.speed_aps;
    float v_rb = -motor_rb->measure.speed_aps;
    float avg_vx_raw = (v_lf + v_rf + v_lb + v_rb) / 4.0f;
    float avg_vy_raw = (-v_lf + v_rf + v_lb - v_rb) / 4.0f;
    float avg_wz_raw = (-v_lf + v_rf - v_lb + v_rb) / 4.0f;
    float geometry_sum = HALF_TRACK_WIDTH + HALF_WHEEL_BASE;
    float wheel_radius_ratio = RADIUS_WHEEL * DEGREE_2_RAD;
    float encoder_vx = avg_vx_raw * wheel_radius_ratio;
    float encoder_vy = avg_vy_raw * wheel_radius_ratio;
    float encoder_wz = avg_wz_raw * (RADIUS_WHEEL / geometry_sum);
    float accel_x = Chassis_IMU_data->Accel[X];
    float accel_y = Chassis_IMU_data->Accel[Y];
    float fused_vx;
    float fused_vy;
    float fused_wz;
    static float imu_vx = 0.0f;
    static float imu_vy = 0.0f;
    static float last_vx = 0.0f;
    static float last_vy = 0.0f;
    static float last_wz = 0.0f;
    const float alpha_linear = 0.05f;
    const float alpha_wz = 0.8f;
    const float drift_correction = 0.02f;
    const float lpf_alpha = 0.3f;

    // IMU 加速度积分提供高频线速度补偿，目的是仅靠编码器逆解算在突加速场景下响应偏慢，融合后能更早反映速度变化趋势。
    imu_vx += accel_x * dt;
    imu_vy += accel_y * dt;

    // 让 IMU 积分值持续向编码器结果收敛，目的是这样能保留 IMU 的高频响应，同时抑制长时间积分漂移。
    imu_vx = imu_vx * (1.0f - drift_correction) + encoder_vx * drift_correction;
    imu_vy = imu_vy * (1.0f - drift_correction) + encoder_vy * drift_correction;

    // 线速度以编码器为主、IMU 为辅做互补滤波，目的是编码器低频更准，IMU 高频更快，组合后比单传感器稳定。
    fused_vx = encoder_vx * (1.0f - alpha_linear) + imu_vx * alpha_linear;
    fused_vy = encoder_vy * (1.0f - alpha_linear) + imu_vy * alpha_linear;

    // 角速度以 IMU 陀螺仪为主、编码器为辅做互补滤波，目的是底盘旋转速度直接看陀螺仪通常更准，但编码器逆解算仍能在局部异常时提供补偿。
    fused_wz = imu_wz_dps * alpha_wz + encoder_wz * (1.0f - alpha_wz);

    // 最后一层低通滤波统一平滑输出，目的是调试或后续控制若直接读取估算速度，不应该被量测毛刺瞬间放大。
    real_vx = (1.0f - lpf_alpha) * last_vx + lpf_alpha * fused_vx;
    real_vy = (1.0f - lpf_alpha) * last_vy + lpf_alpha * fused_vy;
    real_wz = (1.0f - lpf_alpha) * last_wz + lpf_alpha * fused_wz;

    last_vx = real_vx;
    last_vy = real_vy;
    last_wz = real_wz;
}

/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用入口，负责初始化底盘执行器并在任务内串起命令接收、模式控制、运动学解算、功率限制和反馈发送
 *        注意底盘采取右手系，对于平面视图，底盘纵向运动的正前方为 x 正方向；横向运动的右侧为 y 正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis_private.h"

/* 底盘应用包含的模块和信息存储，底盘是单例模式，因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD
CANCommInstance *chasiss_can_comm = NULL; // 双板通信 CANComm 句柄，目的是UI 汇总和任务收命令都要复用同一实例。
attitude_t *Chassis_IMU_data = NULL; // 底盘板 IMU 解算数据，目的是跟随、功率补偿和 UI 都依赖同一份姿态反馈。
#endif // CHASSIS_BOARD

#ifdef ONE_BOARD
static Publisher_t *chassis_pub = NULL; // 单板构型下用于发布底盘反馈，目的是保持单板消息链路仍局限在入口文件内部。
static Subscriber_t *chassis_sub = NULL; // 单板构型下用于订阅底盘控制命令，目的是该句柄只在任务入口消费，不需要外泄。
#endif // ONE_BOARD

Chassis_Ctrl_Cmd_s chassis_cmd_recv; // 当前拍底盘控制命令缓存，目的是主任务和拆分 helper 都围绕同一份命令做增量处理。
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘上传反馈缓存，目的是当前仍由入口任务统一负责最终发送。
referee_info_t *referee_data = NULL; // 裁判系统数据指针，目的是功率限制、超电策略和 UI 汇总都要读同一份裁判状态。
Referee_Interactive_info_t ui_data; // 底盘提供给 UI 任务的实时数据快照，目的是裁判 UI 绘制线程只关心这一份聚合结果。

#ifdef USE_SUPER_CAP
SuperCapInstance *cap = NULL; // 超级电容实例，目的是功率下发、UI 状态和 bonus 策略都要共用同一实例。
#endif

DJIMotorInstance *motor_lf = NULL; // 左前轮电机实例，目的是运动学解算和底盘急停都要直接操作轮组。
DJIMotorInstance *motor_rf = NULL; // 右前轮电机实例，目的是与其余三轮统一纳入同一套功率控制链路。
DJIMotorInstance *motor_lb = NULL; // 左后轮电机实例，目的是运动学解算最终仍需落到具体轮组目标。
DJIMotorInstance *motor_rb = NULL; // 右后轮电机实例，目的是保持四轮参考与启停时序一致。

// 下面这些旧运行时状态仍保留在入口文件内，目的是当前拆分目标仅限核心 helper，下述状态还没有形成明确复用链路，不额外继续扩散。
static PIDInstance yaw_lock_pid; // 航向锁定专用 PID
static float lock_target_yaw = 0.0f; // 锁定的目标角度
static uint8_t is_manual_rotating = 0u; // 标记是否正在手动旋转
static uint8_t last_cali_flag = 0u; // 上一次的校准标志位

float chassis_vx = 0.0f; // 云台系速度投影到底盘坐标系后的 x 分量，目的是主任务负责坐标变换，运动学 helper 直接消费结果即可。
float chassis_vy = 0.0f; // 云台系速度投影到底盘坐标系后的 y 分量，目的是保持运动学 helper 不再关心坐标系转换细节。
static chassis_mode_e last_chassis_mode = CHASSIS_ZERO_FORCE; // 记录上一拍底盘模式，目的是底盘板本地也要能识别小陀螺退跟随边沿，避免上板请求偶发漏拍时接管逻辑失效。
uint8_t last_follow_transition_request = 0u; // 记录接管请求上一拍电平，目的是上板会保持几拍请求位，底盘侧只应在上升沿触发一次接管窗口。
uint8_t last_follow_brake_request = 0u; // 记录跟随刹停请求上一拍电平，目的是V 掉头结束时需要识别刹停请求释放边沿，才能把跟随滤波状态重新贴回当前误差而不是沿用刹停期的零值。
uint8_t follow_transition_ticks = 0u; // 记录当前跟随接管剩余拍数，目的是用固定拍数窗口先刹停再回正，比直接混控更稳且实现确定性更强。
float follow_angle_err_filtered = 0.0f; // 缓存滤波后的跟随偏角，目的是刚退出接管时若直接吃原始偏角，容易被机械回弹和量测毛刺再次拉成反复摆动。
float follow_wz_cmd_limited = 0.0f; // 缓存限斜率后的跟随输出，目的是接管段切回正常跟随时沿用连续状态，避免指令一步跳变刺激轮速环。
uint32_t follow_control_dwt_cnt = 0u; // 记录跟随控制的 DWT 时间基准，目的是输出斜率限制要按真实周期换算，不能假设任务永远严格等于 5ms。

/**
 * @brief 底盘应用初始化
 *
 */
void ChassisInit(void)
{
    Motor_Init_Config_s chassis_motor_config;

    // 底盘上电后先初始化 IMU，目的是后续坡度补偿、跟随阻尼和 UI 都依赖姿态数据，不能等任务阶段再懒初始化。
    Chassis_IMU_data = INS_Init();

    // 四个轮子的控制参数完全一致，只在 CAN ID 和正反转上区分，目的是统一复用一份配置结构能减少重复初始化代码和改参遗漏。
    chassis_motor_config = (Motor_Init_Config_s){
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 3.7,
                .Ki = 0.0,
                .Kd = 0.0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
                .Output_LPF_RC = 0.1,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
        },
        .motor_type = M3508,
    };

    // 轮组采用功率控制初始化入口，目的是当前底盘的速度环目标最终要经过统一功率预算裁剪，不能走普通电机初始化链。
    chassis_motor_config.can_init_config.can_handle = &hcan2;
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rb = PowerControlInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = PowerControlInit(&chassis_motor_config);

#ifdef USE_ISLAND_ACTION
    // 启用上岛机构时在底盘初始化阶段一起拉起，目的是该机构的调度完全挂在底盘任务后段，初始化时机必须与底盘一致。
    IslandActionInit();
#endif // USE_ISLAND_ACTION

    // 裁判系统初始化时顺带把 UI 数据缓冲注册进去，目的是UI 任务后续会直接读取 ui_data 的变化来决定是否刷新图元。
    referee_data = UITaskInit(&huart6, &ui_data);
    // 默认开启坡度补偿，目的是当前功率控制策略已经显式依赖 pitch 角，初始化时直接打开可避免运行初期策略分叉。
    PowerControl_EnableSlopeComp(1);

#ifdef USE_SUPER_CAP
    {
        SuperCap_Init_Config_s cap_conf = {
            .can_config = {
                .can_handle = &hcan2,
                .tx_id = 0x061,
                .rx_id = 0x051,
            }
        };

        // 底盘初始化阶段同步拉起超电实例，目的是后续功率预算、DCDC 使能和 UI 状态都依赖这一实例持续在线。
        cap = SuperCapInit(&cap_conf);
    }
#endif // USE_SUPER_CAP

#ifdef CHASSIS_BOARD
    {
        CANComm_Init_Config_s comm_conf = {
            .can_config = {
                .can_handle = &hcan1,
                .tx_id = 0x011,
                .rx_id = 0x012,
            },
            .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
            .send_data_len = sizeof(Chassis_Upload_Data_s),
            .daemon_count = 200,
        };

        // 双板构型下初始化底盘与云台之间的 CANComm，目的是后续所有命令接收和反馈回传都走这条链路，必须在任务启动前建立。
        chasiss_can_comm = CANCommInit(&comm_conf);

        LOGINFO("[can_comm] Chassis_Upload_Data_s size: %d", sizeof(Chassis_Upload_Data_s));
        LOGINFO("[can_comm] Chassis_Ctrl_Cmd_s size: %d", sizeof(Chassis_Ctrl_Cmd_s));
        LOGINFO("[can_comm] CAN_COMM_MAX_BUFFSIZE: %d", CAN_COMM_MAX_BUFFSIZE);
        if (chasiss_can_comm != NULL) {
            LOGINFO("[CHASSIS_DEBUG] CAN Comm Init Success! TxID: %d, RxID: %d",
                    chasiss_can_comm->can_ins->tx_id,
                    chasiss_can_comm->can_ins->rx_id);
        } else {
            LOGERROR("[CHASSIS_DEBUG] CAN Comm Init Failed!");
        }
    }
#endif // CHASSIS_BOARD

#ifdef ONE_BOARD
    // 单板构型继续通过消息中心对接上层控制链，目的是当前拆分只做底盘模块内聚，不改变单板已有的数据流入口。
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

/* 机器人底盘控制核心任务 */
void ChassisTask(void)
{
    float gimbal_wz = 0.0f;
    uint8_t follow_transition_request_rise = 0u;
    uint8_t follow_brake_request_rise = 0u;
    uint8_t follow_brake_request_fall = 0u;
    uint8_t rotate_to_follow_edge = 0u;
    float referee_power_limit;
    float referee_buffer_energy = DEFAULT_TEST_BUFFER_ENERGY_J;
    uint8_t chassis_output_allowed = 1u;
    float final_power_limit;

    // 先拿到本拍最新的底盘控制命令，目的是后续所有模式切换、功率预算和 UI 汇总都必须基于同一拍输入展开。
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD

    // 先缓存本拍最新的云台角速度前馈，目的是跟随支路要避免“先读旧前馈、再更新命令”造成固定一拍滞后。
    gimbal_wz = chassis_cmd_recv.gimbal_gyro_z;
    // 检测接管请求上升沿，目的是上板会保持数拍请求位，底盘侧只应在真正的边沿触发一次接管窗口。
    follow_transition_request_rise = (uint8_t)(chassis_cmd_recv.follow_transition_request != 0u &&
                                               last_follow_transition_request == 0u);
    // 检测跟随刹停请求上升沿，目的是进入 V 掉头期时需要立刻清空历史跟随状态，避免底盘再被旧滤波和前馈带着走一段。
    follow_brake_request_rise = (uint8_t)(chassis_cmd_recv.follow_brake_request != 0u &&
                                          last_follow_brake_request == 0u);
    // 检测跟随刹停请求释放沿，目的是掉头结束后要把普通跟随滤波状态重新贴齐当前误差，避免从刹停期的零误差直接切回时再抽动。
    follow_brake_request_fall = (uint8_t)(chassis_cmd_recv.follow_brake_request == 0u &&
                                          last_follow_brake_request != 0u);
    // 本地补一份小陀螺退跟随边沿检测，目的是即使上板请求偶发漏拍，底盘也能靠本地模式边沿兜底进入接管。
    rotate_to_follow_edge = (uint8_t)(last_chassis_mode == CHASSIS_ROTATE &&
                                      chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW);
    last_follow_transition_request = chassis_cmd_recv.follow_transition_request;
    last_follow_brake_request = chassis_cmd_recv.follow_brake_request;

    if (chassis_cmd_recv.ui_refresh_request != 0u) {
        // 收到上板的一次性 UI 刷新请求后转交给裁判 UI 任务，目的是真正的绘图发包必须在 UI 线程内串行执行，底盘控制线程不应直接插手。
        UIRequestRefresh();
    }

    // 优先取裁判系统给出的原始功率限制，目的是后续平地/坡道缩放和超电 bonus 都必须建立在合法裁判预算之上。
    if (referee_data == NULL) {
        referee_power_limit = DEFAULT_TEST_POWER;
    } else {
        referee_power_limit = referee_data->GameRobotState.chassis_power_limit;
        if (referee_power_limit < 1.0f) {
            referee_power_limit = DEFAULT_TEST_POWER;
        }
    }

    if (referee_data != NULL) {
        // 同步取一份裁判缓冲能量与输出许可，目的是超电 bonus 需要和同一拍裁判状态对齐，不能只靠上一次的超电板回包做决策。
        referee_buffer_energy = (float)referee_data->PowerHeatData.buffer_energy;
        chassis_output_allowed = (uint8_t)(referee_data->GameRobotState.power_management_chassis_output != 0u);
    }

    // 当前平地与坡道都保持吃满裁判预算，目的是用户明确希望更激进，额外的软件缩放只会白白损失输出。
    if (fabsf(Chassis_IMU_data->Pitch) > CHASSIS_SLOPE_THRESHOLD) {
        final_power_limit = referee_power_limit * 1.00f;
    } else {
        final_power_limit = referee_power_limit * 1.0f;
    }

#ifdef USE_SUPER_CAP
    {
        float super_cap_bonus = GetAggressiveSuperCapBonus(referee_power_limit, referee_buffer_energy, chassis_output_allowed);
        float reported_power_limit = 0.0f;

        if (super_cap_bonus > 0.0f) {
            // 把跟随超电板真实能力算出的动态 bonus 叠加到底盘总功率上限，目的是让底盘不再只吃固定 75W，而是尽量往超电板当前真的能给出的功率区间逼近。
            final_power_limit += super_cap_bonus;
        }

        // 若超电板已经回传了可信的实际可给功率上限，就在底盘侧再做一次保护性裁剪，目的是让轮组功率请求和电源链真实能力保持一致。
        if (cap != NULL &&
            super_cap_policy_state.assist_enabled != 0u &&
            SuperCapIsOnline(cap) != 0u &&
            SuperCapHasHardFault(cap) == 0u &&
            SuperCapIsOutputDisabled(cap) == 0u) {
            reported_power_limit = (float)SuperCapGetReportedPowerLimit(cap);
            if (reported_power_limit >= CHASSIS_SUPER_CAP_REPORTED_LIMIT_MIN_W) {
                super_cap_policy_state.reported_limit_valid = 1u;
                // 这里保留一次最终裁剪，目的是即使 helper 前面已经按 `chassisPowerLimit - 5W` 算了额外功率，本层仍确保总预算不会因为跨拍状态差异而超过同一拍回传上限。
                reported_power_limit -= CHASSIS_SUPER_CAP_REPORTED_LIMIT_MARGIN_W;
                if (final_power_limit > reported_power_limit) {
                    final_power_limit = reported_power_limit;
                }
            } else {
                super_cap_policy_state.reported_limit_valid = 0u;
            }
        } else {
            super_cap_policy_state.reported_limit_valid = 0u;
        }
    }
#endif // USE_SUPER_CAP
    // 把最终预算交给底盘功率控制算法，目的是轮组参考值后续都必须在同一套预算下闭环，不应该各自单独裁剪。
    SetPowerLimit(final_power_limit);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) {
        // 急停或关键模块离线时立即停掉四轮，目的是零力模式的优先级最高，任何剩余控制输出都必须被彻底切断。
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
#ifdef USE_ISLAND_ACTION
        // 零力模式下同步停掉上岛辅助机构，目的是该机构不应在底盘主运动已急停时继续保持动作。
        IslandActionStop();
#endif // USE_ISLAND_ACTION
    } else {
        // 非零力模式下统一使能四轮，目的是让后续模式分支只关心目标值生成，不重复分散处理电机使能。
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据当前底盘模式生成本拍旋转目标，目的是平移坐标变换与运动学解算都依赖这里给出的最终 wz 命令。
    switch (chassis_cmd_recv.chassis_mode) {
    case CHASSIS_NO_FOLLOW:
        ResetAdaptiveSpinBase(); // 退出小陀螺时清空贴边状态，目的是下次重新进入时从统一初始条件起步，避免继承旧工况的高转速记忆。
        ResetFollowControlState(); // 离开底盘跟随时复位接管状态，目的是跟随专用的滤波和限斜率记忆不应泄漏到自由平移模式。
        break;

    case CHASSIS_FOLLOW_GIMBAL_YAW: {
        const float follow_yaw_kp = 21.0f;
        const float follow_yaw_kd = 0.5f;
        const float follow_yaw_kff = 1.3f;
        const float follow_yaw_deadband = 0.5f;
        const float follow_yaw_max_wz = 7500.0f;
        float chassis_wz = Chassis_IMU_data->Gyro[Z] * RAD_2_DEGREE;
        float angle_err = 0.0f;
        float raw_angle_err = chassis_cmd_recv.follow_offset_angle;
        float relative_gimbal_wz = gimbal_wz - chassis_wz;
        float raw_follow_wz = 0.0f;
        float follow_dt_s;

        ResetAdaptiveSpinBase(); // 跟随模式下复位小陀螺状态，目的是跟随控制依赖独立角度环，不应继续带着自旋功率闭环状态运行。
        follow_dt_s = GetFollowControlDt();

        if (follow_brake_request_rise != 0u) {
            // 跟随刹停请求刚进入时立即清空历史滤波、历史输出并重置时间基准，目的是进入 V 掉头期必须第一时间切掉旧跟随残留。
            follow_angle_err_filtered = 0.0f;
            follow_wz_cmd_limited = 0.0f;
            DWT_GetDeltaT(&follow_control_dwt_cnt);
        }

        if (chassis_cmd_recv.follow_brake_request != 0u) {
            // 跟随刹停期间持续把滤波误差压成 0，目的是这一段底盘只允许按自身余旋做阻尼收敛，绝不能再把云台相对角误差重新积回来。
            follow_angle_err_filtered = 0.0f;
            // 跟随刹停期间只按车体自身角速度做阻尼制动，目的是这样既能平滑停住底盘，又能完全隔离云台角速度前馈和位置误差继续把底盘带走。
            raw_follow_wz = -FOLLOW_TRANSITION_BRAKE_KD * chassis_wz;
            LIMIT_MIN_MAX(raw_follow_wz, -FOLLOW_TRANSITION_MAX_WZ, FOLLOW_TRANSITION_MAX_WZ);
        } else {
            if (follow_brake_request_fall != 0u) {
                // 跟随刹停刚释放时把滤波误差贴回当前虚拟前方误差并重置时间基准，目的是掉头完成后恢复普通跟随时必须从当前真实状态平滑接回。
                follow_angle_err_filtered = raw_angle_err;
                DWT_GetDeltaT(&follow_control_dwt_cnt);
            }

            if (follow_transition_request_rise != 0u || rotate_to_follow_edge != 0u) {
                StartFollowTransition(raw_angle_err); // 在小陀螺退跟随边沿启动接管窗口，目的是先刹停再回正，避免残余自旋和偏角环在同一拍里互相打架。
            } else {
                if (last_chassis_mode != CHASSIS_FOLLOW_GIMBAL_YAW) {
                    // 从其它模式首次进入普通跟随时把滤波状态贴齐当前偏角，目的是避免滤波器从 0 起步把第一拍回正量平白压小。
                    follow_angle_err_filtered = raw_angle_err;
                }

                // 对偏角做一阶滤波，目的是先滤掉机械回弹和量测毛刺，退出接管后 P 项不容易马上反向抽动。
                follow_angle_err_filtered += (raw_angle_err - follow_angle_err_filtered) * FOLLOW_ANGLE_FILTER_ALPHA;
                angle_err = follow_angle_err_filtered;
                if (fabsf(angle_err) < follow_yaw_deadband) {
                    // 清零死区内误差，目的是小角度时让前馈和阻尼接管，避免位置项在零点附近反复翻转。
                    angle_err = 0.0f;
                }

                if (follow_transition_ticks != 0u) {
                    // 接管窗口内只按车体角速度做阻尼刹停，目的是先卸掉小陀螺余旋，比同时引入偏角环和前馈更不容易振荡。
                    raw_follow_wz = -FOLLOW_TRANSITION_BRAKE_KD * chassis_wz;
                    LIMIT_MIN_MAX(raw_follow_wz, -FOLLOW_TRANSITION_MAX_WZ, FOLLOW_TRANSITION_MAX_WZ);
                    if (fabsf(chassis_wz) < FOLLOW_TRANSITION_EXIT_WZ_DPS) {
                        // 当余旋已经足够小时提前结束接管，目的是这样可以更早恢复正常回正，减少“刹得过久”的拖滞手感。
                        follow_transition_ticks = 0u;
                    } else {
                        follow_transition_ticks--;
                    }
                } else {
                    // 接管结束后恢复正常跟随控制律，目的是仍保留 P 回正、D 阻尼和相对角速度前馈来兼顾速度与稳定性。
                    raw_follow_wz = -follow_yaw_kp * angle_err -
                                    follow_yaw_kd * chassis_wz -
                                    follow_yaw_kff * relative_gimbal_wz;
                    LIMIT_MIN_MAX(raw_follow_wz, -follow_yaw_max_wz, follow_yaw_max_wz);
                }
            }
        }

        // 对最终跟随输出做限斜率，目的是从接管刹停切回正常回正时保持连续，避免指令跳变再次激起摆振。
        chassis_cmd_recv.wz = ApplyFollowCommandSlew(raw_follow_wz, follow_dt_s);
        break;
    }

    case CHASSIS_ROTATE: {
        float base_wz;

        ResetFollowControlState(); // 进入小陀螺时复位跟随接管状态，目的是小陀螺的控制目标完全不同，不应继续保留跟随环的滤波和输出记忆。
        // 按实时功率余量闭环抬高小陀螺基础角速度，目的是让后级功率限制器长期工作在贴边状态，从而把合法功率尽量吃满。
        base_wz = GetAdaptiveSpinBase(final_power_limit);
        // 在功率贴边基础上继续应用平移优先，目的是小陀螺再猛也不能把驾驶员横移和前后机动直接抢没。
        chassis_cmd_recv.wz = OptimizedSpinSpeed(base_wz, chassis_cmd_recv.vx, chassis_cmd_recv.vy);
        break;
    }

    default:
        ResetAdaptiveSpinBase(); // 其它模式统一复位小陀螺状态，目的是避免未覆盖模式残留旧的小陀螺闭环输出。
        ResetFollowControlState(); // 其它模式统一复位跟随接管状态，目的是任何非跟随工况都不该继续保存跟随专用的边沿和输出历史。
        break;
    }

    {
        static float sin_theta;
        static float cos_theta;

        // 把云台系平移指令映射到底盘坐标系，目的是后续运动学解算固定在底盘坐标系内进行，不能直接混用云台坐标下的 vx、vy。
        cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
        sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
        // 修正右手系下的旋转矩阵符号，目的是offset_angle 以逆时针为正时，sin 项若不按这里的符号处理就会出现推杆前进却横移的现象。
        chassis_vx = chassis_cmd_recv.vx * cos_theta + chassis_cmd_recv.vy * sin_theta;
        chassis_vy = -chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    }

    if (chassis_cmd_recv.chassis_mode == CHASSIS_NO_FOLLOW) {
        // ChassisHeadLock();
    }

    // 先完成底盘运动学解算，再把 pitch/roll 同步给功率控制，目的是保持原有控制链顺序不变，降低拆分后行为回归风险。
    MecanumCalculate();
    // HybridCalculate();
    PowerControl_UpdateIMU(Chassis_IMU_data->Pitch * DEGREE_2_RAD,
                           Chassis_IMU_data->Roll * DEGREE_2_RAD);

    // 根据裁判系统和超电状态对输出限幅并下发轮组目标，目的是这一步之后本拍底盘实际输出就已经确定。
    LimitChassisOutput();

    // 在底盘输出与功率策略完成后刷新一份 UI 实时数据快照，目的是这样 UI 读到的功率、超电和角速度都对应本拍最新控制结果。
    RefereeUIUpdateData();

#ifdef USE_ISLAND_ACTION
    if (chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE) {
        // 在底盘主运动解算后独立控制履带与抬升，同时传入 IMU pitch，目的是上岛辅助机构不参与麦轮功率分配，但自动调平仍要看本拍姿态。
        IslandActionControl(&chassis_cmd_recv, Chassis_IMU_data->Pitch);
    }
#endif // USE_ISLAND_ACTION

    // EstimateSpeed();

#ifdef ONE_BOARD
    // 单板构型下通过消息中心向其它模块推送底盘反馈，目的是拆分 helper 后仍保持原有的反馈出口不变。
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    // 双板构型下通过 CANComm 回传底盘反馈，目的是云台板后续的 UI 和上层逻辑仍依赖这条既有链路。
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD

    // 在任务末尾刷新上一拍模式缓存，目的是下一拍需要用它识别本地的小陀螺退跟随边沿并兜底触发接管。
    last_chassis_mode = chassis_cmd_recv.chassis_mode;
}

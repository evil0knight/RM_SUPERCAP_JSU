#include "robot_cmd_private.h"

#ifdef GIMBAL_BOARD
// 在编译期校验底盘反馈结构体尺寸，目的是双板通信仍需保证单帧 CAN 可以装下完整反馈。
_Static_assert(sizeof(Chassis_Upload_Data_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Upload_Data_s exceeds CAN_COMM_MAX_BUFFSIZE");
// 在编译期校验底盘控制结构体尺寸，目的是新增上岛字段后必须继续保证命令帧不会溢出 CANComm 缓冲区。
_Static_assert(sizeof(Chassis_Ctrl_Cmd_s) <= CAN_COMM_MAX_BUFFSIZE,
               "Chassis_Ctrl_Cmd_s exceeds CAN_COMM_MAX_BUFFSIZE");
// 双板通信句柄在初始化和任务发送阶段都要复用，目的是拆分后不能再把它锁在某个单独文件里。
CANCommInstance *cmd_can_comm = NULL;
#endif

#ifdef ONE_BOARD
// 单板底盘命令发布者需要在初始化和任务主循环间共享，目的是入口文件负责初始化，而主循环负责下发同一条消息链。
Publisher_t *chassis_cmd_pub = NULL;
// 单板底盘反馈订阅者需要在任务主循环里持续取数，目的是拆分后仍然必须围绕同一个订阅实例读取底盘反馈。
Subscriber_t *chassis_feed_sub = NULL;
#endif

// 底盘命令缓存是 cmd 应用每拍叠加写入的核心状态，目的是遥控器、键鼠和恢复链都会往同一份命令里增量写字段。
Chassis_Ctrl_Cmd_s chassis_cmd_send = { 0 };
// 底盘反馈缓存保存当前拍收到的底盘板状态，目的是偏角、功率和 UI 数据都要基于这份最新反馈做决策。
Chassis_Upload_Data_s chassis_fetch_data = { 0 };

// DBUS 遥控数据指针在初始化后全局复用，目的是DT7 分支和键鼠回退分支都会围绕同一个解析缓冲读输入。
RC_ctrl_t *rc_data = NULL;
// 图传键鼠数据指针在初始化后全局复用，目的是VT03 主控和键鼠输入都依赖同一份图传解析结果。
RC_ctrl_t *video_link_data = NULL;

// 云台命令发布者用于把 cmd 结果送到 gimbal 应用，目的是入口和恢复逻辑都会往同一条消息链里写命令。
Publisher_t *gimbal_cmd_pub = NULL;
// 云台反馈订阅者用于拉取最新姿态和电机在线位，目的是偏角计算与恢复逻辑都依赖同一份反馈。
Subscriber_t *gimbal_feed_sub = NULL;
// 云台命令缓存保存当前拍最终要下发给 gimbal 的控制量，目的是遥控器、键鼠、恢复链和掉头逻辑都会覆盖这份命令。
Gimbal_Ctrl_Cmd_s gimbal_cmd_send = { 0 };
// 云台反馈缓存保存当前拍最新姿态与电机状态，目的是偏角解算、恢复同步和自瞄都依赖这份反馈。
Gimbal_Upload_Data_s gimbal_fetch_data = { 0 };

// 发射命令发布者用于把 cmd 结果送到 shoot 应用，目的是遥控器和键鼠分支会共同修改同一份发射命令。
Publisher_t *shoot_cmd_pub = NULL;
// 发射反馈订阅者用于读取射击统计和状态，目的是cmd 层要基于同一份反馈做跨拍火力协同。
Subscriber_t *shoot_feed_sub = NULL;
// 发射命令缓存保存当前拍最终的摩擦轮与装填控制，目的是遥控器、键鼠和急停链都围绕同一份命令做覆盖。
Shoot_Ctrl_Cmd_s shoot_cmd_send = { 0 };
// 发射反馈缓存保存 shoot 应用回传的状态，目的是主循环需要读取统一反馈来维持控制链闭环。
Shoot_Upload_Data_s shoot_fetch_data = { 0 };

// 机器人整体工作状态决定本拍是否进入零力，目的是遥控器、VT03 Pause 和链路故障都要通过这一个状态统一表达。
Robot_Status_e robot_state = ROBOT_STOP;
// 提示蜂鸣器句柄在校零和会话结束时都会复用，目的是拆分后蜂鸣器控制分散到多个文件，但仍要驱动同一个实例。
BuzzzerInstance *hint_buzzer = NULL;

// IMU 测试句柄继续保留原有全局位置，目的是这次只拆入口文件，不改已有调试接口和外部调试习惯。
BMI088Instance *bmi088_test = NULL;
// IMU 测试数据缓存继续保留原有全局位置，目的是当前虽然主循环不主动使用，但保留状态可以避免调试链路被顺手改坏。
BMI088_Data_t bmi088_data = { 0 };

// 保存 DT7 右拨杆上一拍状态，目的是物理挡位切换依赖边沿判断，不能把持续保持误判成新的模式切换。
uint16_t last_switch_right = RC_SW_DOWN;
// 保存 DT7 左拨杆上一拍状态，目的是上岛辅助和发射锁存都依赖左拨杆的中位切换边沿。
uint16_t last_switch_left = RC_SW_DOWN;
// 保存底盘跟随所使用的 yaw 软件对齐基准角，目的是当前链路围绕 DM 硬件零点闭环，但软件仍需保留单一可改的参考角。
float yaw_align_offset_deg = YAW_CHASSIS_ALIGN_DEG;
// 保存底盘跟随模式里被视为“正前方”的真实云台-底盘夹角，目的是V 键掉头完成后需要把新前方向语义跨周期保留下来。
float follow_front_offset_deg = 0.0f;
// 保存遥控器摩擦轮锁存状态，目的是左拨杆上拨需要一拨一切换，而不是按住期间每拍翻转。
uint8_t friction_switch_state = 0u;
// 保存当前遥控器发射模式，目的是现有单发、二连发和连发切换语义必须在拆文件后继续保持。
uint8_t fire_mode_state = 0u;
#ifdef USE_ISLAND_ACTION
// 保存前履带开关锁存状态，目的是上岛辅助机构需要跨控制周期保持启停意图。
uint8_t front_track_switch_state = 0u;
#endif
// 保存 DT7 内八校零是否已在本次组合中触发，目的是校零属于重动作，同一组合里必须只发一次。
uint8_t cali_triggered = 0u;

// 键鼠摩擦轮锁存用于记住 F 键当前是否处于开启态，目的是鼠标左键只负责拨弹，不能再顺带隐式开启摩擦轮。
uint8_t mouse_fire_friction_latched = 0u;
// 保存鼠标左键上一拍电平，目的是左键单发必须严格按边沿触发，按住期间绝不能重复拨弹。
uint8_t mouse_left_last = 0u;
// 保存图传链路上一拍在线状态，目的是图传离线边沿需要第一时间清空旧的键鼠锁存。
uint8_t last_video_link_online = 0u;
// 保存 F 键上一拍电平，目的是键盘摩擦轮是开关语义，只能在真正上升沿触发一次。
uint8_t keyboard_friction_toggle_last = 0u;
// 保存 R 键上一拍电平，目的是 12/16m/s 的切换必须只响应一次上升沿，不能在按住时反复翻转。
uint8_t keyboard_bullet_speed_toggle_last = 0u;
// 保存键鼠当前预选弹速档位，目的是即使暂时关闭摩擦轮，UI 也要持续展示下一次 F 开启后会使用的档位。
Bullet_Speed_e keyboard_bullet_speed_selected = BIG_AMU_12;
// 保存 X 键上一拍电平，目的是小陀螺锁存切换必须依赖上升沿而不是电平。
uint8_t keyboard_spin_toggle_last = 0u;
// 保存 B 键上一拍电平，目的是自由模式锁存切换必须依赖上升沿而不是电平。
uint8_t keyboard_free_toggle_last = 0u;
// 保存 V 键上一拍“确认后电平”，目的是一键掉头属于一次性动作，必须避免按住期间重复触发。
uint8_t keyboard_turnback_toggle_last = 0u;
// 保存 V 键最近一次已经消费过的输入帧序号，目的是控制任务会重复读同一帧，必须先分清是否真的来了新样本。
uint32_t keyboard_turnback_last_frame_serial = 0u;
// 保存 V 键已经连续确认的按下帧数，目的是单帧毛刺或旧帧重读都不能直接触发掉头。
uint8_t keyboard_turnback_press_frame_count = 0u;
// 保存 G 键上一拍电平，目的是UI 整页刷新是一次性请求，不能按住期间反复触发。
uint8_t keyboard_ui_refresh_last = 0u;
// 保存键盘小陀螺锁存状态，目的是用户要求 X 键按一次切一次，而不是按住才进入。
uint8_t keyboard_spin_mode_latched = 0u;
// 保存键盘自由模式锁存状态，目的是用户要求 B 键按一下就持续保持自由模式。
uint8_t keyboard_free_mode_latched = 0u;
// 保存一键掉头执行态，目的是该动作不是普通模式，而是一次持续到完成或被打断的任务。
uint8_t keyboard_turnback_active = 0u;
// 保存本次一键掉头要追踪的累计 yaw 目标，目的是每拍都要稳定追同一个终点，不能反复重新计算。
float keyboard_turnback_target_yaw = 0.0f;
// 保存掉头完成后准备提交的新前方向参考，目的是底盘在掉头期间不动，但动作结束后前方语义必须更新。
float keyboard_turnback_follow_front_target_deg = 0.0f;
// 保存一键掉头开始时间，目的是机构或姿态异常时要靠它做超时保护。
uint32_t keyboard_turnback_start_ms = 0u;
// 保存一键掉头完成稳定拍数，目的是连续多拍满足条件才能认为真正完成。
uint8_t keyboard_turnback_stable_ticks = 0u;
// 保存键盘前后方向的当前平滑输出，目的是`PrepareControlCommandBase` 每拍都会清零瞬态命令，平滑状态必须独立跨拍保存。
float keyboard_vx_smoothed = 0.0f;
// 保存键盘左右方向的当前平滑输出，目的是平移斜坡必须沿用上一拍状态，不能每拍从 0 重新起算。
float keyboard_vy_smoothed = 0.0f;
// 保存键盘平移上次更新时间戳，目的是斜坡推进需要基于真实 dt 计算本拍允许变化量。
uint32_t keyboard_ramp_last_ms = 0u;
// 保存当前主控输入源，目的是整个 cmd 模块都要围绕 VT03 主控、DT7 回退和双离线零力做统一仲裁。
ControlSource_e current_control_source = CONTROL_SOURCE_NONE;
// 保存 VT03 左自定义键上一拍电平，目的是这次把摩擦轮切换改到左 `fn` 后，仍然必须用边沿触发避免按住期间连续翻转。
uint8_t vt03_fn_left_last = 0u;
// 保存 VT03 右自定义键上一拍电平，目的是自定义键是电平输入，必须由 cmd 层自行做上升沿锁存。
uint8_t vt03_fn_right_last = 0u;
// 保存 VT03 扳机上一拍电平，目的是单发拨弹只能响应上升沿，不能把电平直接送进装填状态机。
uint8_t vt03_trigger_last = 0u;
// 保存 VT03 Pause 上一拍电平，目的是Pause 现在同时承担短按零力和长按校零，必须分清按下、保持和释放三个阶段。
uint8_t vt03_pause_last = 0u;
// 保存 VT03 Pause 的零力锁存状态，目的是进入零力后即使松手前链路波动，也不能自动恢复使能。
uint8_t vt03_pause_zero_force_latched = 0u;
// 保存 VT03 Pause 本次按下的起始时间，目的是长按校零必须按真实毫秒时间判定。
uint32_t vt03_pause_press_start_ms = 0u;
// 保存 VT03 Pause 本次按压是否从零力态开始，目的是只有已经零力时的长按才允许触发 yaw 校零。
uint8_t vt03_pause_press_started_in_zero_force = 0u;
// 保存 VT03 Pause 本次按压是否已经处理过长按校零，目的是同一次长按里只能发送一次 DM 校零指令。
uint8_t vt03_pause_longpress_handled = 0u;
// 保存 VT03 首次接管时是否仍需默认进入零力，目的是用户要求上电后 VT03 先失能，但该默认逻辑只能生效一次。
uint8_t vt03_boot_zero_force_pending = 0u;
// 保存 VT03 上一拍挡位编码，目的是换挡时需要贴齐当前姿态，避免切挡瞬间跳变。
uint8_t vt03_mode_sw_last = VT03_MODE_SW_INVALID;
// 保存 pitch 目标同步请求剩余保持拍数，目的是恢复窗口必须保持几拍，才能确保“贴当前姿态”真正送到底层。
uint8_t pitch_target_sync_hold_ticks = 0u;
// 保存这次 pitch 恢复时要贴齐的目标角，目的是恢复窗口内必须反复下发同一个姿态目标。
float pitch_recover_target_deg = 0.0f;
// 保存进入 pitch 标定那一拍锁住的 yaw 目标，目的是标定全过程里云台 yaw 必须停在当前朝向，不能再被遥控器或键鼠增量改写。
float pitch_cali_yaw_lock_target_deg = 0.0f;
// 保存底盘跟随接管请求剩余保持拍数，目的是双板调度可能错开，边沿请求需要保持几拍才稳妥。
uint8_t follow_transition_request_hold_ticks = 0u;
// 保存最后一次可信的真实云台-底盘夹角，目的是yaw 电机离线时要冻结到最近可信值，而不是继续算假偏角。
float last_valid_offset_angle = 0.0f;
// 保存最后一次可信的底盘跟随误差，目的是yaw 离线或掉头刹停期间都要稳定冻结跟随闭环误差。
float last_valid_follow_offset_angle = 0.0f;

// 保存 yaw 电机上一拍在线状态，目的是复活边沿需要把目标贴回当前姿态，而常态下不应反复打断控制。
static uint8_t last_yaw_motor_online = 0u;
// 保存 pitch 电机上一拍在线状态，目的是只有识别到复活边沿，cmd 侧才能触发 pitch 恢复同步。
static uint8_t last_pitch_motor_online = 0u;
// 保存上一拍最终下发到底盘的模式，目的是小陀螺退跟随的请求必须以最终输出层的模式边沿为准。
static chassis_mode_e last_effective_chassis_mode = CHASSIS_ZERO_FORCE;
// 保存上一拍最终下发给云台的模式，目的是只有最终输出层的 ZERO_FORCE -> 有力模式边沿，才能稳定触发 pitch 恢复。
static gimbal_mode_e last_effective_gimbal_mode = GIMBAL_ZERO_FORCE;

/**
 * @brief 将 pitch 目标统一限幅到机构安全范围
 *
 */
void LimitGimbalPitchTarget(void)
{
    // 这里统一约束 pitch 目标，作用是让遥控器和鼠标共用同一份限位；
    // 原因是两个输入源都会改写 pitch，分散限位容易出现一边忘记限位导致撞机构。
    if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE) {
        gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    } else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE) {
        gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;
    }
}

/**
 * @brief 每周期先把控制量恢复到安全基线
 *
 */
static void PrepareControlCommandBase(void)
{
    // 这里先清理瞬态控制量，作用是阻断上一拍残留的速度和发射命令；
    // 原因是键鼠接入后同一份指令会被多源叠加，若不先回到安全基线会出现“松手后还在沿用旧命令”。
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.gimbal_cmd_wz = 0.0f;
    chassis_cmd_send.calibrate_imu = 0;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.cap_mode = SUPER_CAP_OFF;
    chassis_cmd_send.chassis_speed_buff = 0;
    chassis_cmd_send.follow_offset_angle = 0.0f;
    chassis_cmd_send.gimbal_pitch_deg = 0.0f;
    chassis_cmd_send.friction_on = 0u;
    chassis_cmd_send.ui_refresh_request = 0u;
    chassis_cmd_send.follow_transition_request = 0u;
    chassis_cmd_send.follow_brake_request = 0u;
#ifdef USE_ISLAND_ACTION
    chassis_cmd_send.front_track_mode = FRONT_TRACK_OFF;
    chassis_cmd_send.lift_mode = LIFT_OFF;
    chassis_cmd_send.front_track_speed_ref = 0.0f;
    chassis_cmd_send.lift_dial_input = 0.0f;
#endif

    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;

    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    // 每拍都把弹速字段回到空值，目的是彻底斩断上一拍残留的档位，避免调试或 UI 侧误把旧值当成当前有效发射命令。
    shoot_cmd_send.bullet_speed = BULLET_SPEED_NONE;
    shoot_cmd_send.shoot_rate = 0.0f;

}

void RobotCMDInit(void)
{
    // 开机时把 yaw 软件对齐基准直接设为 0 度，目的是让底盘跟随从第一拍就围绕 DM 硬件零点闭环，避免首帧回到旧机械角。
    yaw_align_offset_deg = 0.0f;
    // 开机时把虚拟前方参考也置回 0 度，目的是默认前方必须与云台和底盘的物理回中方向一致，不能带入上次运行期的语义。
    follow_front_offset_deg = 0.0f;
    vt03_boot_zero_force_pending = 0u; //上电不能直接进入零力
    last_yaw_motor_online = 0u;
    last_pitch_motor_online = 0u;
    last_valid_offset_angle = 0.0f;
    last_valid_follow_offset_angle = 0.0f;
    last_effective_chassis_mode = CHASSIS_ZERO_FORCE;
    last_effective_gimbal_mode = GIMBAL_ZERO_FORCE;
    pitch_target_sync_hold_ticks = 0u;
    pitch_recover_target_deg = 0.0f;
    pitch_cali_yaw_lock_target_deg = 0.0f;
    follow_transition_request_hold_ticks = 0u;
    rc_data = RemoteControlInit(&huart3);
    // 将 VT03 图传链路恢复到 USART6，目的是当前实车接线走的是云台板 USART6，挂到 USART1 会导致 VT03 遥控和键鼠都收不到有效帧。
    video_link_data = VideoLinkKMInit(&huart6);
    Buzzer_config_s hint_config = {
        .alarm_level = ALARM_LEVEL_MEDIUM,
        .octave = OCTAVE_5,
        .loudness = 0.5f,
    };
    hint_buzzer = BuzzerRegister(&hint_config);

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif
#ifdef GIMBAL_BOARD
    {
        CANComm_Init_Config_s comm_conf = {
            .can_config = {
                .can_handle = &hcan1,
                .tx_id = 0x012,
                .rx_id = 0x011,
            },
            .recv_data_len = sizeof(Chassis_Upload_Data_s),
            .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
            .daemon_count = 200,
        };
        cmd_can_comm = CANCommInit(&comm_conf);
        LOGINFO("[can_comm] Chassis_Upload_Data_s size: %d", sizeof(Chassis_Upload_Data_s));
        LOGINFO("[can_comm] Chassis_Ctrl_Cmd_s size: %d", sizeof(Chassis_Ctrl_Cmd_s));
        LOGINFO("[can_comm] CAN_COMM_MAX_BUFFSIZE: %d", CAN_COMM_MAX_BUFFSIZE);
        if (cmd_can_comm != NULL) {
            LOGINFO("[GIMBAL_DEBUG] CAN Comm Init Success! TxID: %d, RxID: %d", cmd_can_comm->can_ins->tx_id, cmd_can_comm->can_ins->rx_id);
        } else {
            LOGERROR("[GIMBAL_DEBUG] CAN Comm Init Failed!");
        }
    }
#endif

    robot_state = ROBOT_READY;
}

/**
 * @brief 根据 gimbal app 传回的当前电机角度计算和零位的误差
 *
 */
static void CalcOffsetAngle(void)
{
    static float last_offset_angle = 0.0f;
    float gimbal_angle;
    float align_offset;
    float error;
    float current_offset;

    if (gimbal_fetch_data.yaw_motor_online == 0u) {
        // yaw 电机离线时同时冻结真实夹角和虚拟前方跟随误差，目的是掉电或复活抖动阶段的机械角不可信，继续重算只会同时污染底盘平移坐标变换和跟随闭环。
        chassis_cmd_send.offset_angle = last_valid_offset_angle;
        chassis_cmd_send.follow_offset_angle = last_valid_follow_offset_angle;
        return;
    }

    gimbal_angle = gimbal_fetch_data.yaw_motor_single_round_angle;
    // 使用当前生效的 yaw 软件基准计算底盘跟随偏角，目的是手动校零后继续围绕 0 度闭环，防止 offset 仍引用旧机械安装角。
    align_offset = yaw_align_offset_deg;
    error = gimbal_angle - align_offset;

    while (error < 0.0f)
        error += 360.0f;
    while (error >= 360.0f)
        error -= 360.0f;

    if (error > 180.0f) {
        current_offset = error - 360.0f;
    } else {
        current_offset = error;
    }

    // 在 ±180 度边界加一层迟滞修正，目的是刚从小陀螺或失能退出时，偏角若在边界来回翻面，底盘会出现剧烈抽动。
    if (current_offset > 170.0f && last_offset_angle < -170.0f) {
        current_offset -= 360.0f;
    } else if (current_offset < -170.0f && last_offset_angle > 170.0f) {
        current_offset += 360.0f;
    }

    chassis_cmd_send.offset_angle = current_offset;
    // 用真实夹角减去当前“虚拟前方”参考，得到底盘跟随专用误差，目的是V 键掉头后的新前方向语义只应该影响跟随闭环，不应该改写真实夹角。
    chassis_cmd_send.follow_offset_angle = theta_format(current_offset - follow_front_offset_deg);
    last_offset_angle = current_offset;
    last_valid_offset_angle = chassis_cmd_send.offset_angle;
    last_valid_follow_offset_angle = chassis_cmd_send.follow_offset_angle;
}

/**
 * @brief 紧急停止，包括遥控器急停、重要模块离线和双板通信失效等
 *
 */
void EmergencyHandler(void)
{
    robot_state = ROBOT_STOP;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    // 紧急停止时同步清掉当前拍的发射档位，目的是恢复前任何模块都不该再读到旧的“可发射”弹速配置。
    shoot_cmd_send.bullet_speed = BULLET_SPEED_NONE;
    shoot_cmd_send.shoot_rate = 0.0f;
    friction_switch_state = 0u;
    // 零力出口统一清空键鼠和遥控锁存，目的是否则恢复有力后会把暂停前的旧输入当成当前意图继续执行。
    ResetMouseControlLatchState();
    ResetKeyboardMotionState();
    ResetChassisAuxState();
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
    ControlSource_e active_control_source;

#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    if (gimbal_fetch_data.yaw_motor_online != 0u && last_yaw_motor_online == 0u) {
        // yaw 电机复活第一拍就把目标贴回当前姿态，目的是死亡期间若仍保留旧 yaw 目标，恢复后云台会试图回拉到旧参考导致头发歪。
        SyncGimbalTargetToCurrentAttitude();
    }
    if (gimbal_fetch_data.pitch_motor_online != 0u && last_pitch_motor_online == 0u) {
        // pitch 电机复活第一拍就发起“贴当前姿态”请求，目的是死亡期间旧的 pitch 目标会一直锁存在 cmd 侧，恢复后必须贴回当前姿态而不是被拉到 0 度。
        RequestPitchRecoverToCurrentAttitude();
        LOGINFO("[pitch_recover] source=motor_online");
    }
    last_yaw_motor_online = gimbal_fetch_data.yaw_motor_online;
    last_pitch_motor_online = gimbal_fetch_data.pitch_motor_online;

    // 每拍先清理一次瞬态控制量，作用是让后续遥控器和键鼠都从同一安全基线开始叠加；
    // 原因是当前 `robot_cmd` 已经不是互斥控制源，直接沿用上拍结果会产生残留指令。
    PrepareControlCommandBase();
    // 底盘跟随始终围绕既定零位计算偏角，目的是上电或重连时若把“当前角度”重新写成零位，会直接把真实偏角清掉。
    CalcOffsetAngle();
    active_control_source = GetActiveControlSource();
    HandleControlSourceSwitch(active_control_source);

    // VT03 Pause 锁存后，即使主控短暂切到 DT7 或 NONE 也必须继续零力，目的是用户按下暂停就是明确的失能意图，不能因为链路抖动而自动恢复使能。
    if (vt03_pause_zero_force_latched != 0u && active_control_source != CONTROL_SOURCE_VT03) {
        EmergencyHandler();
    } else if (active_control_source == CONTROL_SOURCE_NONE) {
        // 双输入源都不可用时必须统一回到零力，目的是当前工程把有效主控视为安全前提，缺失主控时不能继续沿用旧命令。
        EmergencyHandler();
    } else {
        // 先收口主遥控源，再叠加当前主控对应的键鼠输入，目的是VT03 遥控器和电脑控制需要同链路共存，同时保留 DT7 断链回退。
        RemoteControlSet(active_control_source);
        MouseKeySet();
    }
    if (last_effective_chassis_mode == CHASSIS_ROTATE &&
        chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
        // 仅在最终有效模式从小陀螺切到底盘跟随时请求接管窗口，目的是只有最终输出层的边沿才覆盖了遥控器、VT03 与键鼠锁存的全部竞争结果。
        RequestFollowTransition();
    }
    if (last_effective_gimbal_mode == GIMBAL_ZERO_FORCE &&
        gimbal_cmd_send.gimbal_mode != GIMBAL_ZERO_FORCE) {
        // 仅在最终有效云台模式从零力恢复到有力时发起 pitch 恢复同步，目的是失能期间保留的旧 pitch 目标必须在恢复第一拍贴回当前姿态。
        RequestPitchRecoverToCurrentAttitude();
        LOGINFO("[pitch_recover] source=gimbal_mode_recover");
    }
    last_effective_chassis_mode = chassis_cmd_send.chassis_mode;
    last_effective_gimbal_mode = gimbal_cmd_send.gimbal_mode;

    if (chassis_cmd_send.chassis_mode != CHASSIS_ZERO_FORCE) {
        // 机器人处于有力模式时默认请求超电介入，目的是用户要求平时超电常开，且底盘侧额外功率策略依赖 cap_mode。
        chassis_cmd_send.cap_mode = SUPER_CAP_ON;
    }
    if (follow_transition_request_hold_ticks != 0u) {
        // 在请求后的若干拍持续下发底盘跟随接管位，目的是双板命令缓冲只保留最新帧，持续几拍才能避免边沿请求被覆盖丢失。
        chassis_cmd_send.follow_transition_request = 1u;
        follow_transition_request_hold_ticks--;
    }
    if (pitch_target_sync_hold_ticks != 0u &&
        gimbal_cmd_send.gimbal_mode != GIMBAL_ZERO_FORCE) {
        // 在最终发包前连续几拍把 pitch 目标强制贴回恢复触发时的当前姿态，目的是只有放在所有输入源都处理完之后，才能确保旧锁存目标和本拍人工输入都压不过这次恢复同步。
        gimbal_cmd_send.pitch = pitch_recover_target_deg;
        LimitGimbalPitchTarget();
        // 这里只继续保持“贴当前姿态”覆盖，不再额外夹带 PID 状态清零，目的是把恢复链简化成单一的目标同步语义。
        pitch_target_sync_hold_ticks--;
    }
    if (GimbalPitchCalibrationActive() != 0u &&
        gimbal_cmd_send.gimbal_mode != GIMBAL_ZERO_FORCE) {
        // pitch 标定运行期间每拍都把 yaw 目标覆盖回进入标定时锁住的那一拍姿态，目的是即使遥控器摇杆、鼠标或键盘还在产生命令，yaw 轴也不能被带离当前朝向。
        gimbal_cmd_send.yaw = pitch_cali_yaw_lock_target_deg;
    }
    // 把云台实时姿态信息显式附带到底盘命令，目的是底盘 UI 和跟随链需要在同一帧里拿到与控制同拍的数据。
    chassis_cmd_send.gimbal_gyro_z = gimbal_fetch_data.gimbal_imu_data.Gyro[2] * RAD_2_DEGREE;
    chassis_cmd_send.gimbal_pitch_deg = gimbal_fetch_data.gimbal_imu_data.Pitch;
    chassis_cmd_send.friction_on = (shoot_cmd_send.friction_mode == FRICTION_ON) ? 1u : 0u;
    // 底盘 UI 优先显示本拍已经明确生效的弹速；若当前没有有效发射档位，再回落到键鼠预选值，保证 F 旁数字既能反映键鼠设置，也不会在遥控发射时停留在旧值。
    if (shoot_cmd_send.bullet_speed != BULLET_SPEED_NONE) {
        chassis_cmd_send.ui_bullet_speed = shoot_cmd_send.bullet_speed;
    } else {
        chassis_cmd_send.ui_bullet_speed = keyboard_bullet_speed_selected;
    }

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}

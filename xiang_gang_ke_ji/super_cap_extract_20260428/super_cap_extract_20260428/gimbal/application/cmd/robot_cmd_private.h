#ifndef ROBOT_CMD_PRIVATE_H
#define ROBOT_CMD_PRIVATE_H

#include "can.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "gimbal.h"

#include "remote_control.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "user_lib.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "buzzer.h"
#include "video_link_km.h"

#include "bsp_dwt.h"
#include "bsp_log.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef GIMBAL_BOARD
// 仅在双板构型下引入 CANComm 类型定义，目的是私有头需要声明 `cmd_can_comm`，但单板构型不应该额外依赖双板通信头。
#include "can_comm.h"
#endif

// 统一声明 `robot_cmd` 拆分后跨编译单元共享的私有常量，目的是这些常量只服务 cmd 应用内部，不应该泄漏到公共头，但拆成多文件后又必须共享同一份定义。
// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)

#define RC_TRIGGER_TH 500
#define YAW_DRIFT_LOCK_COEF 0.95f
#define YAW_DRIFT_LOCK_GYRO_TH_DPS 8.0f
#define BURST_FIRE_RATE 8.0f
#define FRONT_TRACK_SPEED_REF_DEFAULT 10.0f
#define LIFT_SWITCH_UP_INPUT_DEFAULT 0.6f
#define AUX_DIAL_INPUT_MAX 660.0f
#define AUX_DIAL_INPUT_DEADZONE 50.0f
#define MOUSE_YAW_SENSITIVITY_DEG (0.00012f * RAD_2_DEGREE)
#define MOUSE_PITCH_SENSITIVITY_DEG (0.00002925f * RAD_2_DEGREE)
#define REMOTE_PITCH_SENSITIVITY 0.000375f
#define KEYBOARD_CHASSIS_CMD_SCALE 6600.0f
#define KEYBOARD_CHASSIS_RAMP_UP_PER_SEC 22000.0f
#define KEYBOARD_CHASSIS_RAMP_DOWN_PER_SEC 30000.0f
#define KEYBOARD_CHASSIS_CMD_EPSILON 1.0f
#define VT03_MODE_SW_C 0u
#define VT03_MODE_SW_N 1u
#define VT03_MODE_SW_S 2u
#define VT03_MODE_SW_INVALID 0xFFu
#define VT03_PAUSE_CALIB_HOLD_MS 2000u
#define PITCH_TARGET_SYNC_HOLD_TICKS 4u
#define FOLLOW_TRANSITION_REQUEST_HOLD_TICKS 4u
#define TURNBACK_COMPLETE_YAW_ERROR_DEG 6.0f
#define TURNBACK_COMPLETE_STABLE_TICKS 20u
#define TURNBACK_TIMEOUT_MS 3000u
#define TURNBACK_TRIGGER_STABLE_FRAMES 3u
#define RC_DEADZONE 5.0f

// 输入源枚举仍保持为 `robot_cmd` 内部私有类型，目的是该仲裁语义只在 cmd 模块内部使用，没有必要暴露给其它应用层模块。
typedef enum {
    CONTROL_SOURCE_NONE = 0,
    CONTROL_SOURCE_VT03,
    CONTROL_SOURCE_DT7,
} ControlSource_e;

// 双板 CAN 通信句柄在多个拆分文件里都会用到，目的是初始化和任务下发都要复用同一实例，因此改成内部共享声明而不是继续锁在单文件里。
#ifdef GIMBAL_BOARD
extern CANCommInstance *cmd_can_comm;
#endif

// 单板发布订阅句柄需要跨初始化和任务主循环共享，目的是拆分后同一条消息链既会在入口文件里初始化，也会在子文件里消费或写回。
#ifdef ONE_BOARD
extern Publisher_t *chassis_cmd_pub;
extern Subscriber_t *chassis_feed_sub;
#endif

// 下面这些是 cmd 应用长期维护的共享消息缓存，目的是各个拆分文件都在同一拍内围绕这几份命令/反馈做增量修改，必须继续共用同一块状态。
extern Chassis_Ctrl_Cmd_s chassis_cmd_send;
extern Chassis_Upload_Data_s chassis_fetch_data;
extern RC_ctrl_t *rc_data;
extern RC_ctrl_t *video_link_data;
extern Publisher_t *gimbal_cmd_pub;
extern Subscriber_t *gimbal_feed_sub;
extern Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
extern Gimbal_Upload_Data_s gimbal_fetch_data;
extern Publisher_t *shoot_cmd_pub;
extern Subscriber_t *shoot_feed_sub;
extern Shoot_Ctrl_Cmd_s shoot_cmd_send;
extern Shoot_Upload_Data_s shoot_fetch_data;
extern Robot_Status_e robot_state;
extern BuzzzerInstance *hint_buzzer;

// 下面这些是跨控制周期保存的内部运行时锁存，目的是控制源切换、键鼠模式、掉头执行和 VT03 Pause 都依赖稳定的跨拍记忆，拆分后仍要引用同一份状态。
extern uint16_t last_switch_right;
extern uint16_t last_switch_left;
extern float yaw_align_offset_deg;
extern float follow_front_offset_deg;
extern uint8_t friction_switch_state;
extern uint8_t fire_mode_state;
#ifdef USE_ISLAND_ACTION
extern uint8_t front_track_switch_state;
#endif
extern uint8_t cali_triggered;
// 保存键鼠摩擦轮锁存状态，目的是把“只有 F 键能开关摩擦轮”的安全语义固化成跨拍状态，避免鼠标按键再次隐式改写摩擦轮。
extern uint8_t mouse_fire_friction_latched;
// 保存鼠标左键上一拍电平，目的是左键拨弹现在严格按上升沿触发，按住期间不能被重复当成多次单发。
extern uint8_t mouse_left_last;
extern uint8_t last_video_link_online;
extern uint8_t keyboard_friction_toggle_last;
// 保存 R 键上一拍电平，目的是弹速档位切换必须只响应一次上升沿，不能按住期间反复来回跳档。
extern uint8_t keyboard_bullet_speed_toggle_last;
// 保存键鼠当前预选弹速档位，目的是即使摩擦轮关闭，UI 也要稳定显示下次开启后将使用的 12/16m/s 档位。
extern Bullet_Speed_e keyboard_bullet_speed_selected;
extern uint8_t keyboard_spin_toggle_last;
extern uint8_t keyboard_free_toggle_last;
extern uint8_t keyboard_turnback_toggle_last;
extern uint32_t keyboard_turnback_last_frame_serial;
extern uint8_t keyboard_turnback_press_frame_count;
extern uint8_t keyboard_ui_refresh_last;
extern uint8_t keyboard_spin_mode_latched;
extern uint8_t keyboard_free_mode_latched;
extern uint8_t keyboard_turnback_active;
extern float keyboard_turnback_target_yaw;
extern float keyboard_turnback_follow_front_target_deg;
extern uint32_t keyboard_turnback_start_ms;
extern uint8_t keyboard_turnback_stable_ticks;
extern float keyboard_vx_smoothed;
extern float keyboard_vy_smoothed;
extern uint32_t keyboard_ramp_last_ms;
extern ControlSource_e current_control_source;
extern uint8_t vt03_fn_left_last;
extern uint8_t vt03_fn_right_last;
extern uint8_t vt03_trigger_last;
extern uint8_t vt03_pause_last;
extern uint8_t vt03_pause_zero_force_latched;
extern uint32_t vt03_pause_press_start_ms;
extern uint8_t vt03_pause_press_started_in_zero_force;
extern uint8_t vt03_pause_longpress_handled;
extern uint8_t vt03_boot_zero_force_pending;
extern uint8_t vt03_mode_sw_last;
extern uint8_t pitch_target_sync_hold_ticks;
extern float pitch_recover_target_deg;
extern float pitch_cali_yaw_lock_target_deg;
extern uint8_t follow_transition_request_hold_ticks;
extern float last_valid_offset_angle;
extern float last_valid_follow_offset_angle;

// 下面这些函数虽然只在 `robot_cmd` 内部使用，但拆分后要跨多个 `.c` 互相调用，目的是保持私有头集中声明，能避免把内部接口混到公共头里。
void LimitGimbalPitchTarget(void);
void ResetMouseFireState(void);
void ClearKeyboardTurnbackState(void);
uint32_t GetControlSourceKeyFrameSerial(ControlSource_e source);
void ResetMouseControlLatchState(void);
void ResetKeyboardMotionState(void);
void SyncGimbalTargetToCurrentAttitude(void);
void RequestPitchRecoverToCurrentAttitude(void);
void RequestFollowTransition(void);
void ResetVT03PausePressState(void);
void TriggerVT03YawCalibration(void);
uint8_t IsVideoLinkControlReady(void);
ControlSource_e GetActiveControlSource(void);
void HandleControlSourceSwitch(ControlSource_e new_source);
void ResetChassisAuxState(void);
void EmergencyHandler(void);
void RemoteControlSet(ControlSource_e control_source);
void MouseKeySet(void);

#endif // ROBOT_CMD_PRIVATE_H

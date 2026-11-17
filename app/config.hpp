#pragma once

#include <cstdint>

namespace app_config
{
// 一个舵轮由转向 GM6020 和驱动 M3508（C620 电调）组成。
struct module_config
{
    uint8_t bus;             // 0 = CAN1，1 = CAN2。
    uint16_t steer_feedback; // GM6020 反馈 ID：0x205～0x20B。
    uint16_t drive_feedback; // C620 反馈 ID：0x201～0x208。
    float steer_ratio;       // 电机转角 / 舵轮转角的绝对值，必须为正。
    float steer_zero;        // 舵轮朝正前方时的电机轴累计角，rad。
    int8_t steer_sign;       // 舵轮逆时针转时反馈角增大为 +1，否则为 -1。
    int8_t drive_sign;       // 正目标转速能推动车向前为 +1，否则为 -1。
    bool calibrated;         // 完成轮序、ID、零位、传动比和方向核对后设为 true。
};

// 几何顺序固定为左前、右前、左后、右后。保留现有 ID，方向和零位待实测。
// ID 有效即可采集反馈，标定未完成时八台电机保持 relax。
inline constexpr module_config modules[4]{
    {1, 0x206, 0x202, 1.0f, 0.0f, 0, 0, false},
    {0, 0x205, 0x202, 1.0f, 0.0f, 0, 0, false},
    {1, 0x207, 0x201, 1.0f, 0.0f, 0, 0, false},
    {0, 0x206, 0x201, 1.0f, 0.0f, 0, 0, false},
};

// 前 +x、左 +y、逆时针旋转为正。中心到舵轮转轴 0.27 m，对应 x/y 偏移如下。
inline constexpr float module_offset_m = 0.1909188f;
inline constexpr float wheel_radius_m = 0.068f;
inline constexpr float max_translation_mps = 0.5f; // 轮缘线速度上限，m/s。
inline constexpr float max_yaw_radps = 2.0f;       // 车体旋转速度上限，rad/s。
inline constexpr float max_steer_radps = 3.0f;     // 舵轮朝向的目标角速度上限，rad/s。
inline constexpr float stick_deadzone = 0.01f;

struct pid_gains
{
    float kp;
    float ki;
    float kd;
};

// 上机调试初值。位置环输出 rad/s，速度环输出对应电机的 CAN 指令单位。
inline constexpr pid_gains steer_position_pid{5.0f, 0.0f, 0.0f};
inline constexpr pid_gains steer_speed_pid{100.0f, 0.0f, 0.0f};
inline constexpr pid_gains drive_speed_pid{500.0f, 0.0f, 0.0f};

// 调度与容量集中设置。实际循环周期仍受调度和执行耗时影响。
inline constexpr uint32_t control_priority = 5;
inline constexpr uint32_t control_stack_size = 4096;
inline constexpr uint32_t control_period_ticks = 1;
inline constexpr uint32_t remote_queue_size = 128;
inline constexpr uint32_t feedback_queue_size = 64;
inline constexpr float feedback_timeout_s = 0.1f;

static_assert(max_steer_radps > 0.0f, "Steering speed limit must be positive");
static_assert(stick_deadzone >= 0.0f && stick_deadzone < 1.0f);
static_assert(max_translation_mps > 0.0f && wheel_radius_m > 0.0f);
static_assert(control_period_ticks > 0 && remote_queue_size > 1 && feedback_queue_size > 1);
} // namespace app_config

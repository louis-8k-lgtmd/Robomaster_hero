#include "chassis.hpp"
#include "config.hpp"
#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "gm6020.hpp"
#include "m3508.hpp"
#include "dji_manager.hpp"
#include "main.h"

#include <cmath>
#include <cstring>

// 保留原来的在线监视符号。角度是电机轴累计角，单位 rad。
volatile float steer_fl_rad = 0.0f;
volatile float steer_fr_rad = 0.0f;
volatile float steer_rl_rad = 0.0f;
volatile float steer_rr_rad = 0.0f;
volatile uint32_t chassis_can_ready = 0;    // 反馈 ID 配置通过且两条 CAN 初始化成功后为 1。
volatile uint32_t chassis_motion_ready = 0; // 零位、方向等标定配置通过后为 1。
volatile uint32_t chassis_can1_frames = 0;
volatile uint32_t chassis_can2_frames = 0;

namespace chassis
{
namespace
{
using namespace app_config;
constexpr float pi = 3.14159265359f;
bool can_ready = false;
bool enabled = false;

struct feedback_frame
{
    uint8_t bus = 0;   // 来自哪条总线：0 为 CAN1，1 为 CAN2
    uint16_t id = 0;   // CAN 报文 ID，用于区分电机
    uint8_t data[8]{}; // 报文里的 8 字节原始数据
};

feedback_frame feedback_queue[feedback_queue_size]{};
volatile uint32_t feedback_head = 0;
volatile uint32_t feedback_tail = 0;
volatile bool queue_overflow = false;
motor::dji::gm6020 steering[4];
motor::dji::m3508 drive[4];
constexpr bsp::can::handle buses[2]{bsp::can::handle::fdcan1, bsp::can::handle::fdcan2};

bool send_motor(void*, uint8_t bus, const motor::dji::frame_t& frame)
{
    return bus < 2 && bsp::can::transmit(buses[bus], frame.id, frame.data, frame.length) ==
                          pnxtypes::status::ok;
}

motor::dji::manager motors{send_motor};

// 中断只统计和入队。应用线程负责电机解码、PID 和发送，避免同时修改电机状态。
// BSP 收到报文后调用它，它负责统计、筛选、将报文复制进队列，随后由应用线程解析电机反馈。

void can_rx(bsp::can::handle handle, const bsp::can::rx_frame& frame)
{
    if (handle == bsp::can::handle::fdcan1)
    {
        chassis_can1_frames++;
    }
    else if (handle == bsp::can::handle::fdcan2)
    {
        chassis_can2_frames++;
    }
    if (frame.id_kind != bsp::can::id_type::standard ||
        frame.format != bsp::can::frame_mode::classic || frame.len != 8 || frame.id < 0x201 ||
        frame.id > 0x20B)
    {
        return;
    }
    const uint32_t next = (feedback_head + 1U) % feedback_queue_size;
    if (next == feedback_tail)
    {
        queue_overflow = true;
        return;
    }
    auto& destination = feedback_queue[feedback_head];
    destination.bus = (handle == bsp::can::handle::fdcan1 ? 0 : 1);
    destination.id = static_cast<uint16_t>(frame.id);
    // 复制报文信息
    std::memcpy(destination.data, frame.data, sizeof(destination.data));
    // 更新写入位置
    feedback_head = next;
}

// 从队列中读取一个反馈报文，返回是否成功。若队列为空则返回 false。
bool read_feedback(feedback_frame& frame)
{
    // 保存原来的中断状态，暂时屏蔽普通中断，保护队列读取过程。
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool available = feedback_tail != feedback_head;
    if (available)
    {
        frame = feedback_queue[feedback_tail];
        feedback_tail = (feedback_tail + 1U) % feedback_queue_size;
    }
    __set_PRIMASK(mask);
    return available;
}

// 问题排除
bool motor_mapping_ready()
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        const auto& item = modules[i];
        if (item.bus > 1 || item.steer_feedback < 0x205 || item.steer_feedback > 0x20B ||
            item.drive_feedback < 0x201 || item.drive_feedback > 0x208)
        {
            return false;
        }
        for (uint8_t j = 0; j < i; ++j)
        {
            if (item.bus == modules[j].bus &&
                (item.steer_feedback == modules[j].steer_feedback ||
                    item.steer_feedback == modules[j].drive_feedback ||
                    item.drive_feedback == modules[j].steer_feedback ||
                    item.drive_feedback == modules[j].drive_feedback))
            {
                return false;
            }
        }
        if (item.steer_feedback == item.drive_feedback)
        {
            return false;
        }
    }
    return true;
}

// 运动前必须完成标定。反馈采集期间可保留方向为 0，电机维持 relax。
bool calibration_ready()
{
    for (const auto& item : modules)
    {
        if (!item.calibrated || !std::isfinite(item.steer_ratio) || item.steer_ratio <= 0.0f ||
            !std::isfinite(item.steer_zero) || (item.steer_sign != 1 && item.steer_sign != -1) ||
            (item.drive_sign != 1 && item.drive_sign != -1))
        {
            return false;
        }
    }
    return true;
}

float wrap_angle(float value)
{
    while (value > pi)
    {
        value -= pi * 2.0f;
    }
    while (value < -pi)
    {
        value += pi * 2.0f;
    }
    return value;
}

// 四舵轮逆运动学：先把底盘期望速度 (vx, vy, omega) 分解到每个轮中心，
// 再得到轮子的指向角和线速度。这里的 x/y 是轮中心相对底盘中心的位置（米）。
void command_modules(float vx, float vy, float omega)
{
    // speed 是轮缘线速度（m/s），angle 是轮子相对车体前方的目标角（rad）。
    float speed[4]{};
    float angle[4]{};
    float peak = max_translation_mps;
    for (uint8_t i = 0; i < 4; ++i)
    {
        const float x = i < 2 ? module_offset_m : -module_offset_m;
        const float y = (i == 0 || i == 2) ? module_offset_m : -module_offset_m;
        // 平动速度对四轮相同；旋转在 (x,y) 处贡献 (-omega*y, omega*x)。
        const float wheel_x = vx - omega * y;
        const float wheel_y = vy + omega * x;
        speed[i] = std::sqrt(wheel_x * wheel_x + wheel_y * wheel_y);
        angle[i] = std::atan2(wheel_y, wheel_x);
        peak = std::fmax(peak, speed[i]);
    }
    // 若旋转和平移叠加让某个轮超过速度上限，则四轮同比例缩放，保留运动方向。
    const float scale = max_translation_mps / peak;
    for (uint8_t i = 0; i < 4; ++i)
    {
        const auto& item = modules[i];
        // 电机反馈是电机轴累计角；减去前向零位，再除以传动比和方向，得到舵轮角。
        const float current_wheel_angle = (steering[i].fdb.position - item.steer_zero) /
                                          (item.steer_ratio * static_cast<float>(item.steer_sign));
        // 几乎静止时保持当前舵角，避免 atan2(0,0) 导致空转向。
        float delta = speed[i] > 0.001f ? wrap_angle(angle[i] - current_wheel_angle) : 0.0f;
        float wheel_speed = speed[i] * scale;
        // 若转向超过 90 度，则舵角反转 180 度并反转驱动轮速，以缩短转向时间。
        if (delta > pi / 2.0f)
        {
            delta -= pi;
            wheel_speed = -wheel_speed;
        }
        else if (delta < -pi / 2.0f)
        {
            delta += pi;
            wheel_speed = -wheel_speed;
        }
        // 转向使用串级 PID：位置环以电机轴角度误差生成目标角速度，
        // 速度环再根据实测角速度生成 GM6020 电压指令；实现见 dji_motor.cpp。
        steering[i].motor_mode = motor::mode::pos_vel;
        steering[i].cmd.position =
            steering[i].fdb.position + delta * item.steer_ratio * item.steer_sign;
        // 驱动轮要持续转动，故仅使用速度环；线速度除以轮半径得到角速度 rad/s。
        drive[i].motor_mode = motor::mode::vel;
        // M3508 驱动已将反馈换算到减速器输出轴。轮轴直连输出轴时，目标也用该轴的 rad/s。
        drive[i].cmd.velocity = wheel_speed * item.drive_sign / wheel_radius_m;
    }
}

// 左拨杆低档或通信异常时，全部电机进入 relax；motors.update() 会把输出清零。
void relax_modules()
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        steering[i].motor_mode = motor::mode::relax;
        drive[i].motor_mode = motor::mode::relax;
    }
}
} // namespace

void init()
{
    can_ready = motor_mapping_ready();
    enabled = can_ready && calibration_ready();
    chassis_motion_ready = enabled ? 1U : 0U;
    if (!can_ready)
    {
        return;
    }
    // health_tick() 使用名义循环周期累计反馈年龄；实际循环耗时仍需上机测量。
    motors.set_period(static_cast<float>(control_period_ticks) / TX_TIMER_TICKS_PER_SECOND);
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (!motors.register_motor(steering[i], modules[i].bus, modules[i].steer_feedback) ||
            !motors.register_motor(drive[i], modules[i].bus, modules[i].drive_feedback))
        {
            Error_Handler();
        }
        // 以下仅是低速初值；先架空车轮检查方向，再按实测反馈调 PID。
        steering[i].position_pid.tune(steer_position_pid.kp, steer_position_pid.ki,
            steer_position_pid.kd);
        steering[i].speed_pid.tune(steer_speed_pid.kp, steer_speed_pid.ki, steer_speed_pid.kd);
        drive[i].speed_pid.tune(drive_speed_pid.kp, drive_speed_pid.ki, drive_speed_pid.kd);
        // 位置环输出是电机轴目标角速度。按传动比换算，同时保留原有上限。
        auto& position_pid = steering[i].position_pid;
        position_pid.max_out =
            std::fmin(position_pid.max_out, max_steer_radps * modules[i].steer_ratio);
    }
    // CAN 回调必须在总线初始化前注册；CAN1 控左侧，CAN2 控右侧。
    if (bsp::can::register_rx_callback(bsp::can::handle::fdcan1,
            bsp::can::rx_callback::bind<&can_rx>()) !=
            pnxtypes::status::ok || // 登记收到数据后的处理函数
        bsp::can::register_rx_callback(bsp::can::handle::fdcan2,
            bsp::can::rx_callback::bind<&can_rx>()) != pnxtypes::status::ok ||
        bsp::can::init(bsp::can::config{bsp::can::handle::fdcan1}) !=
            pnxtypes::status::ok || // 启动 CAN 总线
        bsp::can::init(bsp::can::config{bsp::can::handle::fdcan2}) != pnxtypes::status::ok)
    {
        Error_Handler();
    }
    chassis_can_ready = 1;
}

bool receive()
{
    feedback_frame frame{};
    while (read_feedback(frame))
    {
        (void)motors.receive(frame.bus, frame.id, frame.data);
    }
    steer_fl_rad = steering[0].fdb.position;
    steer_fr_rad = steering[1].fdb.position;
    steer_rl_rad = steering[2].fdb.position;
    steer_rr_rad = steering[3].fdb.position;
    // 临界区只取走溢出标志并清队列，电机解析放在临界区外。
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool overflow = queue_overflow;
    queue_overflow = false;
    if (overflow)
    {
        feedback_tail = feedback_head;
    }
    __set_PRIMASK(mask);
    return !overflow;
}

void update(float vx, float vy, float omega, bool allow_motion)
{
    if (!can_ready)
    {
        return;
    }
    bool feedback_ready = true;
    for (uint8_t i = 0; i < 4; ++i)
    {
        feedback_ready &= steering[i].feedback_age() <= feedback_timeout_s &&
                          drive[i].feedback_age() <= feedback_timeout_s;
    }
    if (allow_motion && enabled && feedback_ready &&
        bsp::can::current_status(buses[0]) == pnxtypes::status::ok &&
        bsp::can::current_status(buses[1]) == pnxtypes::status::ok)
    {
        command_modules(vx, vy, omega);
    }
    else
    {
        relax_modules();
    }
    motors.update();
    motors.transmit(static_cast<uint32_t>(bsp::dwt::timeline_us()));
}
} // namespace chassis

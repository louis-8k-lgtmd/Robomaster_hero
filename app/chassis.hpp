#pragma once

namespace chassis
{
// 仅由应用线程调用。底盘独占 CAN1/CAN2 的电机收发和八个电机对象。
// 总线和 ID 有效即可收反馈，完成标定后才允许运动。
void init();

// 在应用线程消费 CAN 反馈并更新角度。队列溢出时丢弃积压数据并返回 false。
bool receive();

// vx、vy 单位 m/s，omega 单位 rad/s。allow_motion 为 false 时发送 relax 零输出。
// 内部还检查标定、电机反馈时效和 CAN 状态，然后解算、运行 PID、发送分组报文。
void update(float vx, float vy, float omega, bool allow_motion);
} // namespace chassis

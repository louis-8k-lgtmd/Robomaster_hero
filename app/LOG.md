# app 应用层与底盘上机调试记录

本目录用于 H723 的 DR16 四舵轮底盘。转向是 GM6020，驱动是 M3508 配 C620。
用户已上机确认 DR16 有效帧、三个控制摇杆通道和左拨杆三档正常。
电机反馈、零位、方向、PID 和整车运动仍需在新固件上验证。

## 1. 入口和执行流程

| 文件 | 职责和入口 |
|---|---|
| [app.cpp](app.cpp) | 唯一 app_start()，创建应用线程 run()，读取输入、接收反馈、映射摇杆、决定低档停机、更新底盘。 |
| [config.hpp](config.hpp) | 集中保存电机映射、标定、几何、限速、PID、线程和队列参数。 |
| [remote.cpp](remote.cpp)、[remote.hpp](remote.hpp) | init() 配置 UART5 RX DMA，read() 组帧和交付最新输入，invalidate() 清除旧指令。 |
| [chassis.cpp](chassis.cpp)、[chassis.hpp](chassis.hpp) | init() 注册电机和 CAN，receive() 解码反馈，update() 检查状态、解算、PID、发帧。 |

启动链为板级 main() → ThreadX → BSP tx_application_define() → app_start() → run()。
主循环是 remote::read() → chassis::receive() → vx/vy/omega 映射 → chassis::update()。
应用只有一个控制线程，remote 和 chassis 由它驱动。后续服务在 app_start() 显式启动，
当前未启动 IMU 和云台。

UART5 RX DMA 由 remote 独占，两条 CAN 的电机收发由 chassis 独占。中断只复制数据入队，
应用线程负责解析和更新电机。共享队列使用短临界区，设备访问和 PID 在临界区之外。
监视变量是调试副本，多变量采样可能跨更新时刻，需要一致快照时暂停目标后读取。

遥控帧超时按此前调试要求仍关闭，停止收帧后沿用最后输入。上电未收到有效帧、左拨杆低档、
接收队列溢出、任一电机反馈过期或 CAN 异常会使输出保持或切换为 relax 零指令。

## 2. 修改参数的位置

下面参数统一在 [config.hpp](config.hpp)，改完重新编译烧录。标定参数由用户记录和填写。

| 参数 | 当前值或填写方法 | 验证方式 |
|---|---|---|
| modules[0..3] | 左前、右前、左后、右后。现有 ID 保留，方向为 0，calibrated=false。 | 每次只转一个轮，看对应角度。 |
| bus | 0 是 CAN1，1 是 CAN2。 | 核对实物接线和总线计数。 |
| steer_feedback | GM6020 的 0x205～0x20B，等于 0x204 + 拨码 ID。 | 逐台抓包确认。 |
| drive_feedback | C620 的 0x201～0x208，等于 0x200 + 电调 ID。 | 同总线反馈 ID 互不重复，建议 C620 使用 ID 1～4。 |
| steer_ratio | 电机转角变化量 / 舵轮转角变化量的绝对值，当前 1。 | 测舵轮已知转角和电机变化量。 |
| steer_zero | 舵轮朝前时电机轴累计角，rad。 | 用相同姿态重启复查。 |
| steer_sign | 从上方看舵轮逆时针转时，反馈角增大为 +1，减小为 -1。 | 连续收反馈时手动转舵。 |
| drive_sign | 车轮向能推动车前进的方向滚动，反馈转速为正填 +1，否则 -1。 | 保持已标记的前向姿态核对转速。 |
| calibrated | 轮序、ID、比值、零位和方向核对完成才设 true。 | chassis_motion_ready。 |
| module_offset_m | 0.1909188 m，方形底盘中心到舵轮转轴的 x/y 分量，0.27 / sqrt(2)。 | 量实车前后和左右偏移。 |
| wheel_radius_m | 0.068 m。 | 滚动一圈距离 / 2π。 |
| max_translation_mps | 0.5 m/s，同时限制解算后的轮缘目标速度。 | 低速架空后落地验证。 |
| max_yaw_radps | 1 rad/s，车体旋转目标限速。 | 原地旋转和组合输入。 |
| max_steer_radps | 3 rad/s，舵轮朝向目标转速，按传动比换算成位置环输出上限。 | 对照转向速度目标和反馈。 |
| stick_deadzone | 0.05，范围 [0,1)。 | 中位输出零，轻拨平滑。 |
| steer_position_pid | Kp/Ki/Kd = 5/0/0。 | 转向角误差、超调、目标角速度。 |
| steer_speed_pid | Kp/Ki/Kd = 100/0/0。 | 转向电机目标和反馈角速度。 |
| drive_speed_pid | Kp/Ki/Kd = 500/0/0。 | M3508 输出轴目标和反馈角速度。 |
| control_priority / control_stack_size | 5 / 4096 字节。 | 调度和栈容量。 |
| control_period_ticks | 1 tick，管理器按此名义周期累计反馈年龄。 | 实测实际循环时间，改周期时同步核对 PID。 |
| remote_queue_size / feedback_queue_size | 128 字节 / 64 帧，实际各少一个槽。 | 排查溢出时先看调度和暂停。 |
| feedback_timeout_s | 0.1 s，8 台电机都需有新反馈才能运动。 | 架空断开反馈测试。 |

M3508 类使用 3591/187 ≈ 19.20 的减速比，反馈已换算到减速器输出轴。
轮轴直连输出轴时，目标使用轮缘线速度 / 轮半径。若输出轴与轮轴之间还有外部传动，
才在 command_modules() 的驱动速度换算处额外乘以输出轴角速度 / 轮轴角速度。

## 3. 先接收反馈，再标定

1. 架空，左拨杆低档，核对接线供电，DR16 可先单独验证。
2. 核对四组总线和反馈 ID，保留方向为 0、calibrated=false。
3. 烧录后 chassis_can_ready 应为 1，两条总线计数增长，手动转轮时对应角度变化。
4. 连续收反馈时转过已知舵角，测 steer_ratio 和 steer_sign。长时间暂停时不要转动机构，跨圈累计需要连续反馈。
5. 断电，将轮子滚动方向对齐车身前后并做标记。重新上电保持不动，记录四个前向角填入 steer_zero。
6. 在该标记姿态下滚动驱动轮，核对转速符号并填 drive_sign。
7. 从不同初始舵角重启复查零位。若有电机整圈歧义，需补回零或绝对舵角测量。
8. 完成核对后把四组 calibrated 改 true，重新烧录。低档先确认零输出，再中档小幅架空验证。

| 角位 | 总线 | 转向反馈 ID | 驱动反馈 ID | 转向比 | 正前角 rad | 转向符号 | 驱动符号 | 确认 |
|---|---|---|---|---|---|---|---|---|
| 左前 | CAN1 | 0x206 | 0x202 | 1（待实测） | | | | ☐ |
| 右前 | CAN2 | 0x205 | 0x202 | 1（待实测） | | | | ☐ |
| 左后 | CAN1 | 0x207 | 0x201 | 1（待实测） | | | | ☐ |
| 右后 | CAN2 | 0x206 | 0x201 | 1（待实测） | | | | ☐ |

## 4. 在线监视和 PID 调参

烧录和调试器加载同一份新 ELF，重新刷新符号，旧地址会变化。

| 全局变量 | 用途 |
|---|---|
| dr16_left_x/y、dr16_right_x/y | 归一化摇杆。 |
| dr16_left_switch | 0/1/2 对应低/中/上档。 |
| dr16_decoded_frames | 成功解码帧计数。 |
| dr16_rx_stage | 0 未初始化，1 UART 初始化中，2 DMA 已启动。 |
| dr16_rx_bytes、dr16_last_rx_size | 累计接收字节和最近回调大小。 |
| dr16_rejected_windows | 被拒绝的候选窗口数。 |
| dr16_rx_chunks_18、dr16_raw_18[0..17] | 18 字节回调块计数及最近一块原始数据。 |
| chassis_can_ready | ID 配置通过且两条 CAN 初始化成功。 |
| chassis_motion_ready | 标定配置检查通过，运动仍需输入、反馈和总线状态允许。 |
| chassis_can1_frames、chassis_can2_frames | BSP 回调收帧计数。 |
| steer_fl_rad、steer_fr_rad、steer_rl_rad、steer_rr_rad | 四轮转向电机轴累计角 rad，顺序为左前、右前、左后、右后。 |

DR16 收字节却不能解码时，在 remote::read() 看返回值和原始数据。此前已兼容滚轮字段为 0
的中位读数，其他字段仍校验。总线计数增长而角度不动时，在 chassis::receive() 核对 ID。
低层反馈解析见 [dji_motor.cpp](../devices/motor/dji/src/dji_motor.cpp)，PID 算法见
[pid.hpp](../libs/control/include/pid.hpp)。

调参顺序是转向速度内环 → 转向位置外环 → 驱动速度环，每次只改一个参数。

| 现象 | 优先调整 |
|---|---|
| 速度跟踪慢且输出未饱和 | 小幅提高对应速度环 Kp。 |
| 速度振荡 | 减速度环 Kp，若加 I 后出现则先减 Ki。 |
| 稳定后持续速度误差且输出未饱和 | P 调稳后小幅加 Ki，观察超调和停机后的残余积分。 |
| 小角度转舵慢，内环跟踪正常且未触及限速 | 小幅提高位置环 Kp。 |
| 大角度打舵过快 | 降 max_steer_radps。 |
| 位置来回摆 | 内环稳定后减位置环 Kp，位置 Ki 先保持 0。 |
| 加 D 后输出尖峰或高频抖动 | 减 Kd 或退回 0，检查噪声和目标突变。 |
| 长期饱和仍跟不上 | 降测试目标，检查供电、机械、反馈和限幅。 |

用 DAP/ST-Link 观察目标、反馈和输出，动态过程用连续采样曲线。单独调转向内环时需专用
受限速度测试入口，正常 command_modules() 会设置 pos_vel。Ki/Kd 吸收采样周期影响，
加入 I/D 前先量实际周期。relax 输出零但不自动清历史积分，使用非零 I 时须验证恢复表现。
需要各轮独立 PID 时，将三组配置改成按轮序的数组，在 chassis::init() 按下标取值。

## 5. 条件修改的代码入口

| 条件 | 入口及同步范围 |
|---|---|
| 整车遥控方向相反 | app.cpp 的 run() 内 vx/vy/omega 符号，单轮异常先查映射和方向参数。 |
| 恢复遥控超时或增加解锁 | remote::read() 提供帧时效，run() 的 allow_motion 组织条件，失效后等待新输入。 |
| 改长方形底盘 | config 增前后/左右偏移，command_modules() 的 x/y 分别引用。 |
| 转向行程受限或上电角度歧义 | calibration_ready()、command_modules() 接入限位、回零或绝对角有效性。 |
| 转舵未对准就驱动产生偏移 | command_modules() 根据优化后舵角误差收紧驱动速度。 |
| 收紧积分或输出限幅 | chassis::init() 设置实例的 max_out/max_iout。GM6020 最终上限 15000，C620 为 16384，均为指令单位。 |
| 非零 I 时恢复跳变 | 应用状态切换处通过 PID reset_state() 重置历史，复测恢复。 |
| CAN 换口或自动恢复 | chassis.cpp 的 buses、can_rx()、init()/update()。按 BSP stop/restart 接口明确恢复条件。 |
| DR16 换串口 | 用户 JSON 的资源选择、remote::init() 和板级 IOC，顺序为 configure → init → start_rx_to_idle。 |
| 加 IMU、裁判等业务 | app_start() 显式启动现有服务，应用循环读取结果，遵守总线所有权和线程约束。 |

H723 FDCAN 使用经典 CAN 标准帧、8 字节数据、BRS 关闭、标称 1 Mbit/s。
配置源为 [板级 IOC](../boards/cubemx/h723_mc02/h723_mc02.ioc)，收发为
[BSP fdcan.cpp](../bsp/can/src/fdcan.cpp)。GM6020 电压组为 0x1FF/0x2FF，
C620 电流组为 0x200/0x1FF，打包见 [dji_manager.cpp](../devices/motor/dji/src/dji_manager.cpp)。

## 6. 构建与验收

```powershell
cmake --preset h723_mc02
cmake --build --preset h723_mc02
```

固件为 build/h723_mc02/embedded_framework.elf，F407 构建不包含本应用。
此次重构已检查 H723 构建，重新烧录后需复验 DR16、CAN 反馈和控制行为。

- [ ] 映射有效、标定未完成时可收角度且全部 relax。
- [ ] 左拨杆低档时推动摇杆仍输出零。
- [ ] 架空验证前后、横移、旋转的轮序和方向。
- [ ] 回中时驱动目标为零、转向目标跟随当前角度。
- [ ] 架空验证电机反馈中断或 CAN 异常时输出零。
- [ ] 恢复遥控超时后验证断开遥控停机。
- [ ] 重启零位可复现。
- [ ] 架空通过后低速落地验证单轴与组合运动。

| 日期 / 固件版本 | 项目 | 修改前 → 修改后 | 实测数据 / 现象 | 结果和下一步 |
|---|---|---|---|---|
| | | | | |
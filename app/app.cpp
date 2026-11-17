#include "config.hpp"
#include "chassis.hpp"
#include "remote.hpp"

#include "bsp_dwt.hpp"
#include "main.h"
#include "tx_api.h"

#include <cmath>

namespace
{
TX_THREAD control_thread{};
alignas(8) uint8_t control_stack[app_config::control_stack_size]{};
char control_name[] = "app";

// 应用的唯一控制线程。输入解析、电机反馈和 PID 均在这个线程执行。
void run(ULONG)
{
    if (bsp::dwt::init() != pnxtypes::status::ok)
    {
        Error_Handler();
    }
    remote::init();
    chassis::init();

    while (true)
    {
        remotes::reading input{};
        bool input_valid = remote::read(input);
        bool received_state = chassis::receive();
        if (received_state == false)
        {
            remote::invalidate();
            input_valid = false;
        }

        // 车体坐标为前 +x、左 +y、逆时针旋转为正。
        const float vx = input.left_y * app_config::max_translation_mps;
        const float vy = -input.left_x * app_config::max_translation_mps;
        const float omega = -input.right_x * app_config::max_yaw_radps;
        const bool allow_to_motion = input_valid && (input.left_sw != remotes::sw_state::low);
        chassis::update(vx, vy, omega, allow_to_motion);
        tx_thread_sleep(app_config::control_period_ticks);
    }
}
} // namespace

// BSP 在 tx_application_define() 中调用一次。后续应用服务也在这里显式启动。
extern "C" void app_start()
{
    if (tx_thread_create(&control_thread, control_name, run, 0, control_stack,
            sizeof(control_stack), app_config::control_priority, app_config::control_priority,
            TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        Error_Handler();
    }
}

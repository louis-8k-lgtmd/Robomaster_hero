#include "remote.hpp"
#include "config.hpp"
#include "bsp_usart.hpp"
#include "memory.hpp"
#include "user_config.hpp"
#include "main.h"

#include <cstring>

// 在线调试器读取的副本。应用控制使用 latest，避免依赖监视变量。
volatile float dr16_left_x = 0.0f;
volatile float dr16_left_y = 0.0f;
volatile float dr16_right_x = 0.0f;
volatile float dr16_right_y = 0.0f;
volatile uint32_t dr16_left_switch = 0;
volatile uint32_t dr16_decoded_frames = 0;   // 成功解码的帧数，每成功一帧加 1
volatile uint32_t dr16_rx_stage = 0;         // 0：未进入线程，1：初始化中，2：已启动 RX DMA。
volatile uint32_t dr16_rx_bytes = 0;         // 接收回调累计交来的字节数
volatile uint32_t dr16_rejected_windows = 0; // 尝试解析但校验失败的 18 字节窗口数
volatile uint32_t dr16_last_rx_size = 0;     // 最近一次接收回调交来的字节数
volatile uint32_t dr16_rx_chunks_18 = 0;     // 回调大小恰好为 18 字节的次数
volatile uint8_t dr16_raw_18[remotes::dr16::frame_size]{}; //	最近一个 18 字节回调块的原始内容

namespace remote
{
namespace
{
using namespace app_config;
// H723 UART DMA 使用可访问 SRAM 和按缓存行对齐的缓冲区，缓存维护由 BSP 完成。
memory::buffer<64> remote_dma RAM_D1_BSS;
uint8_t remote_queue[remote_queue_size]{};
volatile uint32_t remote_head = 0;
volatile uint32_t remote_tail = 0;
volatile bool queue_overflow = false;
uint8_t candidate[remotes::dr16::frame_size]{};
uint32_t candidate_size = 0;
remotes::reading latest{};
bool valid = false;

void remote_rx(bsp::usart::handle, uint8_t* bytes, uint32_t size)
{
    dr16_rx_bytes = dr16_rx_bytes + size;
    dr16_last_rx_size = size;
    // 仅保存正好 18 字节的回调块，供在线查看。组帧仍由底盘线程完成。
    if (size == remotes::dr16::frame_size)
    {
        for (uint32_t i = 0; i < size; ++i)
        {
            dr16_raw_18[i] = bytes[i]; // raw 的18个字节是最近一次回调的原始内容，调试器内可以查看
        }
        dr16_rx_chunks_18 = dr16_rx_chunks_18 + 1U;
    }
    // 循环 DMA 在回调返回后可覆盖原缓冲区，所以必须在中断内先复制数据。
    for (uint32_t i = 0; i < size; ++i)
    {
        const uint32_t next = (remote_head + 1U) % remote_queue_size;
        if (next == remote_tail)
        {
            // 不覆盖尚未读取的字节；线程看到溢出标志后会清空队列并让遥控失效。
            queue_overflow = true;
            return;
        }
        remote_queue[remote_head] = bytes[i];
        remote_head = next;
    }
}

// 从接收队列中读取一个字节，返回是否成功。若队列为空则返回 false。
bool read_remote_byte(uint8_t& byte)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool available = remote_tail != remote_head;
    if (available)
    {
        byte = remote_queue[remote_tail];
        remote_tail = (remote_tail + 1U) % remote_queue_size;
    }
    __set_PRIMASK(mask);
    return available;
}

} // namespace

void init()
{
    dr16_rx_stage = 1;
    if (bsp::usart::configure(user_config::dr16_uart.instance, user_config::dr16_line) !=
        pnxtypes::status::ok)
    {
        Error_Handler();
    }
    if (bsp::usart::init(user_config::dr16_uart, bsp::usart::mode::dma) != pnxtypes::status::ok)
    {
        Error_Handler();
    }
    if (bsp::usart::start_rx_to_idle(user_config::dr16_uart.instance, remote_dma,
            bsp::usart::rx_callback::bind<&remote_rx>()) != pnxtypes::status::ok)
    {
        Error_Handler();
    }
    dr16_rx_stage = 2;
}

// 用来丢弃当前积压的遥控数据，并把遥控输入标记为无效，让程序等待新的有效帧。
void invalidate()
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    queue_overflow = false;
    remote_tail = remote_head;
    __set_PRIMASK(mask);
    candidate_size = 0;
    valid = false;
}

// 拼帧过程与调用解码器、保存结果，并把最新遥控数据交给调用者，也有解码的过程！！！
bool read(remotes::reading& input)
{
    // DMA 可能交来半帧或多帧。失败时滑动一字节，重新寻找 18 字节帧边界。
    uint8_t byte = 0;
    while (read_remote_byte(byte))
    {
        candidate[candidate_size++] = byte;
        if (candidate_size == remotes::dr16::frame_size)
        {
            if (remotes::dr16::decode(candidate, candidate_size, latest) == pnxtypes::status::ok)
            {
                valid = true;
                dr16_left_x = latest.left_x;
                dr16_left_y = latest.left_y;
                dr16_right_x = latest.right_x;
                dr16_right_y = latest.right_y;
                dr16_left_switch = static_cast<uint32_t>(latest.left_sw);
                dr16_decoded_frames++;
                candidate_size = 0;
            }
            else
            {
                // 滑动窗口找帧边界
                dr16_rejected_windows++;
                std::memmove(candidate, candidate + 1, candidate_size - 1);
                --candidate_size;
            }
        }
    }
    if (queue_overflow)
    {
        invalidate();
    }
    input = latest;
    return valid;
}
} // namespace remote

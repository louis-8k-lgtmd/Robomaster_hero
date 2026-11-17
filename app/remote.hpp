#pragma once

#include "remoter.hpp"

namespace remote
{
// 仅由应用线程调用。UART5 RX DMA 由本文件对应实现独占，不创建后台线程。
void init();

// 消费 UART 队列并复制最新输入。成功解码前返回 false；队列溢出后等待新帧。
// 遥控帧超时判断按当前调试要求关闭，停止收帧时继续返回最近一次有效值。
bool read(remotes::reading& input);

// 丢弃旧输入和排队字节。用于应用故障后等待下一帧有效数据。
void invalidate();
} // namespace remote

#include "dji_manager.hpp"

namespace motor::dji
{

bool manager::register_motor(motor& instance, uint8_t bus, uint16_t feedback_id) noexcept
{
    if (bus >= buses || feedback_id < 0x201 || feedback_id > 0x20B)
    {
        return false;
    }
    auto& slot = motors_[bus][feedback_id - 0x201];
    if (slot)
    {
        return false;
    }
    slot = &instance;
    return true;
}

bool manager::receive(uint8_t bus, uint16_t id, const uint8_t data[8]) noexcept
{
    if (!data || bus >= buses || id < 0x201 || id > 0x20B)
    {
        return false;
    }
    auto* m = motors_[bus][id - 0x201];
    if (!m)
    {
        return false;
    }
    m->receive(data);
    return true;
}

void manager::update()
{
    for (auto& bus : motors_)
    {
        for (auto* m : bus)
        {
            if (m)
            {
                m->health_tick(period_);
                m->update();
            }
        }
    }
}

void manager::transmit(uint32_t now_us)
{
    if (!sender_)
    {
        return;
    }
    for (uint8_t bus = 0; bus < buses; ++bus)
    {
        for (uint8_t group = 0; group < groups; ++group)
        {
            if (!tx_[bus][group].due(now_us))
            {
                continue;
            }
            frame_t frame{};
            frame.id = command_ids[group];
            bool populated = false;
            for (uint8_t i = 0; i < 4; ++i)
            {
                const uint8_t index = group * 4 + i;
                auto* m = index < 11 ? motors_[bus][index] : nullptr;
                if (!m)
                {
                    continue;
                }
                populated = true;
                const auto raw = static_cast<uint16_t>(m->output());
                frame.data[2 * i] = static_cast<uint8_t>(raw >> 8);
                frame.data[2 * i + 1] = static_cast<uint8_t>(raw);
            }
            if (populated && sender_(context_, bus, frame))
            {
                tx_[bus][group].mark_sent(now_us);
            }
        }
    }
}

} // namespace motor::dji

#pragma once
#include "motor_manager.hpp"
#include "motor_utils.hpp"
#include "dji_motor.hpp"
#include <cstdint>

namespace motor::dji
{
struct frame_t
{
    uint16_t id = 0;
    uint8_t data[8]{};
    uint8_t length = 8;
};

using send_fn = bool (*)(void* context, uint8_t bus, const frame_t& frame);

class manager final : public manager_interface
{
public:
    explicit manager(send_fn sender, void* context = nullptr) noexcept
        : sender_(sender), context_(context)
    {
    }

    bool register_motor(motor& instance, uint8_t bus, uint16_t feedback_id) noexcept;
    bool receive(uint8_t bus, uint16_t id, const uint8_t data[8]) noexcept;
    void update() override;

    void set_period(float seconds) noexcept
    {
        period_ = seconds;
    }

    bool set_tx_period(uint8_t bus, uint16_t id, uint32_t us) noexcept
    {
        if (bus >= buses)
        {
            return false;
        }
        for (uint8_t group = 0; group < groups; ++group)
        {
            if (id == command_ids[group])
            {
                return tx_[bus][group].set_period(us);
            }
        }
        return false;
    }

    void transmit(uint32_t now_us) override;

private:
    static constexpr uint8_t buses = 3;
    static constexpr uint8_t groups = 3;
    static constexpr uint16_t command_ids[groups]{0x200, 0x1FF, 0x2FF};
    motor* motors_[buses][11]{};
    tx_schedule tx_[buses][groups]{};
    float period_ = 0.001f;
    send_fn sender_ = nullptr;
    void* context_ = nullptr;
};
} // namespace motor::dji

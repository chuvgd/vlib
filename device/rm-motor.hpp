/**
 * @file rm-motor.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-18
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
 * usage example:
 * 
 * app         gimabal_task    chassis_task    command_task    detect_task
 * --------------------------------------------------------------------------------------
 *                 ^            ^              ^                        ^           ^
 *             yaw_motor    pitch_motor    wheel_motor                  |           |
 *             ------------------------    ---------------------        |           |
 * middleware  rm_motor_manager_can1       rm_motor_manager_can2    superCap    upper2lower
 *             ------------------------    ---------------------------------    -----------
 *             can1(or fdcan1)             can2(or fdcan2)                      uart1
 * --------------------------------------------------------------------------------------
 *                 ^                                                                ^
 * bsp         bsp_can                                                          bsp_uart
*/

#ifndef VLIB_DEVICE_RM_MOTOR_HPP
#define VLIB_DEVICE_RM_MOTOR_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace vlib {

template<size_t PORT>
class RM_Motor {
    static std::array<uint8_t, 8> txbuf;

public:
    uint8_t id;

    int16_t encoder;      // 2pi/8191
    int16_t velocity;     // rpm
    int16_t current_fdb;  // 20A/16384
    uint8_t temperature;  // C
    int16_t current_ctrl; // 20A/16384

    RM_Motor(uint8_t id_):
        id(id_) {}

    void Decode(std::array<uint8_t, 8>& buf) {
        encoder     = (int16_t)((buf[0] << 8) | buf[1]);
        velocity    = (int16_t)((buf[2] << 8) | buf[3]);
        current_fdb = (int16_t)((buf[4] << 8) | buf[5]);
        temperature = buf[6];
    }

    void SetCurrent(int16_t current) {
        current_ctrl                  = current;
        txbuf[((id - 1) % 4) * 2]     = current_ctrl >> 8;
        txbuf[((id - 1) % 4) * 2 + 1] = current_ctrl;
    }

    std::array<uint8_t, 8>& Flush() { return txbuf; }

    static constexpr float M3508_GEAR_RATIO = 3591.f / 187.f;
    static constexpr float M2006_GEAR_RATIO = 36.f;

    static constexpr uint32_t M3508_TX_ID_1 = 0x200;
    static constexpr uint32_t M2006_TX_ID_1 = 0x200;
    static constexpr uint32_t M3508_TX_ID_2 = 0x1FF;
    static constexpr uint32_t M2006_TX_ID_2 = 0x1FF;

    uint32_t M3508_RX_ID() const { return 0x200 + id; }
    uint32_t M2006_RX_ID() const { return 0x200 + id; }

    static constexpr uint32_t GM6020_TX_V_ID_1 = 0x1FF;
    static constexpr uint32_t GM6020_TX_V_ID_2 = 0x2FF;
    static constexpr uint32_t GM6020_TX_A_ID_1 = 0x1FE;
    static constexpr uint32_t GM6020_TX_A_ID_2 = 0x2FE;
    uint32_t                  GM6020_RX_ID() const { return 0x204 + id; }

    float Angle() { return (float)encoder * 2.f * 3.14159265f / 8191.f; }
    float Velocity() { return (float)velocity / 60.f * 2.f * 3.14159265f; }
    float Current() { return (float)current_fdb * 20.f / 16384.f; }
    float Temperature() { return (float)temperature; }

    static constexpr float HEXROLL_GEAR_RATIO = 268.f / 17.f;

    float HexRoll_Velocity() { return Velocity() / HEXROLL_GEAR_RATIO; }

    int16_t HexRoll_Torque2Current(float torque) { return (int16_t)(torque / HEXROLL_GEAR_RATIO); }

    int16_t Torque2Current(float torque) { return (int16_t)(torque / M3508_GEAR_RATIO / 0.3f * 16384.f / 20.f); }
};

} // namespace vlib

#endif

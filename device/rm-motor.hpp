/**
 * @file rm-motor.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 大疆电机（M3508 / M2006 / GM6020）
 * @version 0.1.3
 * @date 2026-05-18
 *
 * @copyright Copyright (c) Serialist 2026
 *
 * 用法：
 *   vlib::RM_Motor<1>        wheel {1};   // 总线1，0x200 组
 *   vlib::RM_Motor<1, 0x1FF> gm    {1};   // 总线1，0x1FF 组（与上面互不冲突）
 *
 *   wheel.SetCurrent(1000);
 *   bsp::can1 << vlib::RM_Motor<1>::Flush();
 */

#ifndef VLIB_DEVICE_RM_MOTOR_HPP
#define VLIB_DEVICE_RM_MOTOR_HPP

#include <array>
#include <stddef.h>
#include <stdint.h>

#include "vlib/bsp/can.hpp"

namespace vlib {

/**
 * @brief 大疆电机
 *
 * @tparam PORT  总线编号（同一总线 = 同一个静态发送缓冲）
 * @tparam TX_ID 控制帧 ID（0x200 / 0x1FF / 0x2FF / 0x1FE / 0x2FE ...）
 *
 * 关键：TX_ID 参与类型，所以不同 TX_ID 的电机拥有各自独立的 txbuf，
 *       同一条总线上多个控制组不会互相踩。
 */
template<size_t PORT, uint32_t TX_ID>
class RM_Motor {
public:
    uint8_t id = 0;

    int16_t encoder      = 0; // 2pi/8191
    int16_t velocity     = 0; // rpm
    int16_t current_fdb  = 0; // 20A/16384
    uint8_t temperature  = 0; // C
    int16_t current_ctrl = 0; // 20A/16384

    explicit RM_Motor(uint8_t id_):
        id(id_) {}

    /// @brief 解析一帧反馈
    void Decode(const std::array<uint8_t, 8>& buf) {
        encoder     = (int16_t)((buf[0] << 8) | buf[1]);
        velocity    = (int16_t)((buf[2] << 8) | buf[3]);
        current_fdb = (int16_t)((buf[4] << 8) | buf[5]);
        temperature = buf[6];
    }

    /// @brief 设置本电机在当前控制帧中的电流（写入共享 txbuf 的槽位）
    void SetCurrent(int16_t current) {
        current_ctrl                  = current;
        txbuf[((id - 1) % 4) * 2]     = (uint8_t)(current_ctrl >> 8);
        txbuf[((id - 1) % 4) * 2 + 1] = (uint8_t)(current_ctrl & 0xFF);
    }

    /// @brief 把本组（TX_ID）当前周期的命令打包成一帧
    static CAN_Frame<8> Flush() {
        CAN_Frame<8> f;
        f.id   = TX_ID;
        f.data = txbuf;
        return f;
    }

    // ---------------- 常量与换算 ----------------
    static constexpr float M3508_GEAR_RATIO   = 3591.f / 187.f;
    static constexpr float M2006_GEAR_RATIO   = 36.f;
    static constexpr float HEXROLL_GEAR_RATIO = 268.f / 17.f;

    static constexpr uint32_t M3508_TX_ID_1    = 0x200;
    static constexpr uint32_t M3508_TX_ID_2    = 0x1FF;
    static constexpr uint32_t M2006_TX_ID_1    = 0x200;
    static constexpr uint32_t M2006_TX_ID_2    = 0x1FF;
    static constexpr uint32_t GM6020_TX_V_ID_1 = 0x1FF;
    static constexpr uint32_t GM6020_TX_V_ID_2 = 0x2FF;
    static constexpr uint32_t GM6020_TX_A_ID_1 = 0x1FE;
    static constexpr uint32_t GM6020_TX_A_ID_2 = 0x2FE;

    uint32_t M3508_RX_ID() const { return 0x200 + id; }
    uint32_t M2006_RX_ID() const { return 0x200 + id; }
    uint32_t GM6020_RX_ID() const { return 0x204 + id; }

    float Angle() const { return (float)encoder * 2.f * 3.14159265f / 8191.f; }
    float Velocity() const { return (float)velocity / 60.f * 2.f * 3.14159265f; }
    float Current() const { return (float)current_fdb * 20.f / 16384.f; }
    float Temperature() const { return (float)temperature; }

    float HexRoll_Velocity() const { return Velocity() / HEXROLL_GEAR_RATIO; }

    static int16_t HexRoll_Torque2Current(float torque) { return (int16_t)(torque / HEXROLL_GEAR_RATIO); }
    static int16_t Torque2Current(float torque) { return (int16_t)(torque / M3508_GEAR_RATIO / 0.3f * 16384.f / 20.f); }

private:
    static std::array<uint8_t, 8> txbuf;
};

// 类外定义（必须在 namespace vlib 内、类之后）
template<size_t PORT, uint32_t TX_ID>
std::array<uint8_t, 8> RM_Motor<PORT, TX_ID>::txbuf {};

} // namespace vlib

#endif

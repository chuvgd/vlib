/**
 * @file supercap.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-03-29
 *
 * @copyright Copyright (c) Serialist 2026
 *
 * 参考 [【RM】VGD25超电CAN通信文档](https://hongliu.icu/2025/09/SuperCap-CAN/)
 * P_referee = P_chassis + P_cap
 */

#ifndef SUPERCAP_H
#define SUPERCAP_H

#include "array"
#include "stdbool.h"
#include "stdint.h"

namespace vilb {
namespace device {

class SuperCap {
public:
    constexpr static uint8_t SUPERPOWER_FDB_ID = 0x211;
    constexpr static uint8_t SUPERPOWER_CMD_ID = 0x210;

    std::array<uint8_t, 8> buf; // 发送缓存
    uint16_t cnt;               // 离线看门狗

    float output;       // 底盘功率
    float cappower_fdb; // 电容目标功率
    float source;       // 电管输入功率
    float capvoltage;   // 电容电压
    float cappower_cmd; // 电容目标功率

    SuperCap(void) = default;
    SuperCap(void): cnt(0) {}

    /// @brief 超电反馈解码
    /// @param buf
    /// @param data
    void Feedback_Decode(uint8_t* buf) {
        uint16_t output_ = (buf[1] << 8 | buf[0]);
        uint16_t cappower_fdb_ = (buf[3] << 8 | buf[2]);
        uint16_t source = (buf[5] << 8 | buf[4]);
        uint16_t capvoltage_ = (buf[7] << 8 | buf[6]);

        output = (output_ - 0x7FFF) / 10.0f;
        cappower_fdb = (cappower_fdb_ - 0x7FFF) / 10.0f;
        source = (source - 0x7FFF) / 10.0f;
        capvoltage = (capvoltage_) / 100.0f;
        cnt = 50;
    }

    /// @brief 超电控制编码
    /// @param data
    /// @param buf
    /// @note @note buf[2..7] 保留位
    void Control_Encode(void) {
        uint16_t cappower_cmd_ = cappower_cmd * 10 + 0x7FFF;

        buf[0] = cappower_cmd_ & 0xFF;
        buf[1] = (cappower_cmd_ >> 8) & 0xFF;
        buf[2] = 0;
        buf[3] = 0;
        buf[4] = 0;
        buf[5] = 0;
        buf[6] = 0;
        buf[7] = 0;
    }

    /// @brief 在线检测
    void Monitor(void) {
        cnt = (cnt > 0) ? cnt-- : 0;
    }

    bool IsOnline(void) {
        return cnt > 0;
    }
};

} // namespace device
} // namespace vilb

#endif /* SUPERCAP_HPP */

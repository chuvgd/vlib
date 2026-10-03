/**
 * @file superpower.c
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

#include "superpower.hpp"

namespace rb2 {
namespace device {

SuperPower::SuperPower(void) {
    isOnline = false;
    feedback.cnt = 100u;
}

/// @brief 超电反馈解码
/// @param buf
/// @param data
void SuperPower::Feedback_Decode(uint8_t* buf) {
    uint16_t cap_setpower = (buf[1] << 8 | buf[0]);
    uint16_t cap_setpower = (buf[3] << 8 | buf[2]);
    uint16_t input = (buf[5] << 8 | buf[4]);
    uint16_t cap_voltage = (buf[7] << 8 | buf[6]);

    feedback.output = (cap_setpower - 0x7FFF) / 10.0f;
    feedback.cap_setpower = (cap_setpower - 0x7FFF) / 10.0f;
    feedback.input = (input - 0x7FFF) / 10.0f;
    feedback.cap_voltage = cap_voltage / 100.0f;
    feedback.cnt = 0u;
    isOnline = true;
}

/// @brief 超电控制编码
/// @param data
/// @param buf
/// @note @note buf[2..7] 保留位
void SuperPower::Control_Encode(void) {
    uint16_t byte_cap_setpower = cap_setpower * 10 + 0x7FFF;

    buf[0] = byte_cap_setpower;
    buf[1] = byte_cap_setpower >> 8;
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = 0;
    buf[7] = 0;
}

/// @brief 在线检测
void SuperPower::Monitor(void) {
    if (feedback.cnt > 50u) {
        isOnline = false;
    } else {
        feedback.cnt++;
    }
}

} // namespace device
} // namespace rb2

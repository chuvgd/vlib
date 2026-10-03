/**
 * @file superpower.h
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-03-29
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef SUPERPOWER_H
#define SUPERPOWER_H

#include "stdbool.h"
#include "stdint.h"

namespace rb2 {
namespace device {

class SuperPower {
public:
#define SUPERPOWER_FDB_ID 0x211
#define SUPERPOWER_CMD_ID 0x210

    struct Feedbak {
        float output;       // 底盘功率
        float cap_setpower; // 电容目标功率
        float input;        // 电管输入功率
        float cap_voltage;  // 电容电压
        uint8_t cnt;
    } feedback;
    float cap_setpower;
    bool isOnline; // 超电在线标志位
    uint8_t buf[8];

    SuperPower(void) = default;

    /// @brief 超电反馈解码
    /// @param buf
    /// @param data
    void Feedback_Decode(uint8_t* buf);

    /// @brief 超电控制编码
    /// @param data
    /// @param buf
    void Control_Encode(void);

    /// @brief 在线检测
    void Monitor(void);
};

} // namespace device
} // namespace rb2

#endif

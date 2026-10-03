/**
 * @file dm-motor.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 达妙电机
 * @version 0.1.0
 * @date 2026-05-08
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef DM_MOTOR_HPP
#define DM_MOTOR_HPP

#include <stdint.h>

namespace rb2 {
namespace device {

class DM_Motor {
public:
    using CanID = uint32_t;
    using MasterID = uint32_t;

    // 控制模式
    enum class Mode : uint32_t {
        MIT = 0x000,
        Position = 0x100,
        Speed = 0x200,
    };

    // 状态
    enum class State : uint8_t {
        Disable = 0x0,              // 失能
        Enable = 0x1,               // 使能
        OverVoltage = 0x8,          // 过压
        UnderVoltage = 0x9,         // 欠压
        OverCurrent = 0xA,          // 过流
        OverTemperatureMOS = 0xB,   // mos温度过高
        OverTemperatureRotor = 0xC, // 电机温度过高
        CommunicationLost = 0xD,    // 通讯丢失
        Overload = 0xE              // 过载
    };

    // 缩放参数
    struct Parameter {
        float p_min, p_max;
        float v_min, v_max;
        float kp_min, kp_max;
        float kd_min, kd_max;
        float t_min, t_max;
    };

    // 反馈数据
    struct Feedback {
        uint32_t id; // 控制器的 ID，取 CAN_ID 的低 8 位
        State state; // 状态

        float position; // 位置
        float velocity; // 速度
        float torque;   // 扭矩

        uint8_t temperature_mos;   // 驱动上 MOS 的平均温度
        uint8_t temperature_rotor; // 电机内部线圈的平均温度
    };

    struct Control {
        float kp, kd;             // pd 系数
        float position, velocity; // 目标位置和速度
        float torque;             // 前馈力矩
    };

    enum class Command : uint8_t {
        Enable = 0xFC,     // 使能
        Disable = 0xFD,    // 失能
        SetZero = 0xFE,    // 设置零点
        ClearError = 0xFB, // 清除错误状态
    };

    const Parameter dm4310 {
        -12.5f, 12.5f, -30.0f, 30.0f, 0.0f, 500.0f, 0.0f, 5.0f, -10.0f, 10.0f
    };

    DM_Motor();

    void SetCommand(Command cmd);
    void Transmit(Control data);
    void Receive(uint8_t* buf);

private:
    CanID can_id;
    MasterID master_id;
    Mode mode;
    Parameter param;

    Feedback feedback;
    Control control;
    uint8_t buf[8];

    void Feedback_Decode(uint8_t* data);
    void MIT_Encode(float kp, float kd, float position, float velocity, float torque);
    void PosSpd_Encode(float position, float velocity);
    void Speed_Encode(float velocity);

    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    int float_to_uint(float x_float, float x_min, float x_max, int bits);
};

} // namespace device
} // namespace rb2

#endif

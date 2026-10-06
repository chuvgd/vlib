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

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace vlib {
namespace comm {

class DM_Motor {
public:
    using CanID    = uint32_t;
    using MasterID = uint32_t;

    // 控制模式
    enum class Mode : uint32_t {
        MIT      = 0x000,
        Position = 0x100,
        Speed    = 0x200,
    };

    // 状态
    enum class State : uint8_t {
        Disable              = 0x0, // 失能
        Enable               = 0x1, // 使能
        OverVoltage          = 0x8, // 过压
        UnderVoltage         = 0x9, // 欠压
        OverCurrent          = 0xA, // 过流
        OverTemperatureMOS   = 0xB, // mos温度过高
        OverTemperatureRotor = 0xC, // 电机温度过高
        CommunicationLost    = 0xD, // 通讯丢失
        Overload             = 0xE  // 过载
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
        uint32_t id;                  // 控制器的 ID，取 CAN_ID 的低 8 位
        State state = State::Disable; // 状态

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
        Enable     = 0xFC, // 使能
        Disable    = 0xFD, // 失能
        SetZero    = 0xFE, // 设置零点
        ClearError = 0xFB, // 清除错误状态
    };

    DM_Motor(Parameter param_):
        param(param_) {}

    void SetCommand(Command cmd) {
        buf = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, static_cast<uint8_t>(cmd) };
    }

    void Transmit(Control data) {}

    void Receive(uint8_t* rx) {}

private:
    CanID can_id       = 0;
    MasterID master_id = 0;
    Mode mode          = Mode::MIT;
    Parameter param;

    Feedback feedback {};
    Control control {};
    std::array<uint8_t, 8> buf {};

    void Feedback_Decode(uint8_t* rx) {
        feedback.id    = rx[0] & 0x0F;
        feedback.state = (State)(rx[0] >> 4);

        int position_int = (rx[1] << 8) | rx[2];
        int velocity_int = (rx[3] << 4) | (rx[4] >> 4);
        int torque_int   = ((rx[4] & 0xF) << 8) | rx[5];

        feedback.temperature_mos   = rx[6];
        feedback.temperature_rotor = rx[7];

        feedback.position = uint_to_float(position_int, param.p_min, param.p_max, 16);
        feedback.velocity = uint_to_float(velocity_int, param.v_min, param.v_max, 12);
        feedback.torque   = uint_to_float(torque_int, param.t_min, param.t_max, 12);
    }

    void MIT_Encode(float kp, float kd, float position, float velocity, float torque) {
        uint8_t data[8];
        uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

        pos_tmp = float_to_uint(position, param.p_min, param.p_max, 16);
        vel_tmp = float_to_uint(velocity, param.v_min, param.v_max, 12);
        kp_tmp  = float_to_uint(kp, param.kp_min, param.kp_max, 12);
        kd_tmp  = float_to_uint(kd, param.kd_min, param.kd_max, 12);
        tor_tmp = float_to_uint(torque, param.t_min, param.t_max, 12);

        data[0] = (pos_tmp >> 8);
        data[1] = pos_tmp;
        data[2] = (vel_tmp >> 4);
        data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        data[4] = kp_tmp;
        data[5] = (kd_tmp >> 4);
        data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
        data[7] = tor_tmp;
    }

    void PosSpd_Encode(float position, float velocity) {
        uint8_t data[8];
        uint32_t p, v;

        std::memcpy(&p, &position, sizeof(p));
        std::memcpy(&v, &velocity, sizeof(v));

        data[0] = (p >> 0) & 0xFF;
        data[1] = (p >> 8) & 0xFF;
        data[2] = (p >> 16) & 0xFF;
        data[3] = (p >> 24) & 0xFF;

        data[4] = (v >> 0) & 0xFF;
        data[5] = (v >> 8) & 0xFF;
        data[6] = (v >> 16) & 0xFF;
        data[7] = (v >> 24) & 0xFF;
    }

    void Speed_Encode(float velocity) {
        uint8_t data[8];
        uint32_t v;

        std::memcpy(&v, &velocity, sizeof(v));

        data[0] = (v >> 0) & 0xFF;
        data[1] = (v >> 8) & 0xFF;
        data[2] = (v >> 16) & 0xFF;
        data[3] = (v >> 24) & 0xFF;
    }

    int float_to_uint(float x_float, float x_min, float x_max, int bits) {
        float span   = x_max - x_min;
        float offset = x_min;
        return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
    }

    float uint_to_float(int x_int, float x_min, float x_max, int bits) {
        float span   = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }
};

static constexpr DM_Motor::Parameter dm4310 { -12.5f, 12.5f, -30.0f, 30.0f,  0.0f,
                                              500.0f, 0.0f,  5.0f,   -10.0f, 10.0f };

} // namespace comm
} // namespace vlib

#endif

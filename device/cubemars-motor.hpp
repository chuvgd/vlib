/**
 * @file cubemars-motor.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-13
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef CUBEMARS_MOTOR_HPP
#define CUBEMARS_MOTOR_HPP

#include "array"
#include "cstdint"

namespace vlib {
namespace comm {

class AK_Motor {
public:
    enum class Mode { MIT, SERVO };
    struct Param {
        float pmax, vmax, kpmin, kpmax, kdmin, kdmax, tmax;
    };

    const Param& param;
    const Mode mode;

    uint8_t id {};
    float position {};
    float velocity {};
    float torque {};
    float temp {};
    float errorcode {};
    uint8_t cnt {}; // 掉线看门狗

    AK_Motor(const AK_Motor::Param& param_, const Mode mode_):
        param(param_),
        mode(mode_),
        cnt(0) {}

    void Enable(void) { // 据说使能命令最好不要重复发送...?
        std::array<uint8_t, 8> buf { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
    }

    void Disable(void) {
        std::array<uint8_t, 8> buf { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
    }

    void Setorigin(void) { // 但这个似乎断电不保存...要小心...
        std::array<uint8_t, 8> buf { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
    }

    std::array<uint8_t, 8> Control_Encode(float pos, float vel, float kp, float kd, float torque) {
        auto angle_int    = Float2Bit(pos, -param.pmax, param.pmax, 16);
        auto velocity_int = Float2Bit(vel, -param.vmax, param.vmax, 12);
        auto kp_int       = Float2Bit(kp, param.kpmin, param.kpmax, 12);
        auto kd_int       = Float2Bit(kd, param.kdmin, param.kdmax, 12);
        auto torque_int   = Float2Bit(torque, -param.tmax, param.tmax, 12);

        std::array<uint8_t, 8> buf;
        buf[0] = angle_int >> 8;
        buf[1] = angle_int & 0xFF;
        buf[2] = velocity_int >> 4;
        buf[3] = ((velocity_int & 0xF) << 4) | (kp_int >> 8);
        buf[4] = kp_int & 0xFF;
        buf[5] = kd_int >> 4;
        buf[6] = ((kd_int & 0xF) << 4) | (torque_int >> 8);
        buf[7] = torque_int & 0xFF;
        return buf;
    }

    void Feedback_Decode(std::array<uint8_t, 8> buf) {
        id       = buf[0];
        position = Bit2Float((int)(buf[1] << 8) | (buf[2]), -param.pmax, param.pmax, 16);
        velocity = Bit2Float((int)(buf[3] << 4) | (buf[4] >> 4), -param.vmax, param.vmax, 12);
        torque   = Bit2Float((int)((buf[4] & 0xF) << 8) | (buf[5]), -param.tmax, param.tmax, 12);
        temp     = (int)buf[6] - 40;
        cnt      = 50u;
    }

    void Monitor(void) {
        cnt = cnt > 0 ? cnt - 1 : 0;
    }

    bool IsOnline(void) {
        return cnt > 0;
    }

    float Bit2Float(int x_int, float x_min, float x_max, int Bits) {
        return ((float)x_int) * (x_max - x_min) / ((float)((1 << Bits) - 1)) + x_min;
    }

    int Float2Bit(float x, float x_min, float x_max, int bits) {
        x = x < x_min ? x_min : x > x_max ? x_max : x; // 限幅
        return (int)((x - x_min) * ((float)((1 << bits) - 1)) / (x_max - x_min));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PacketID_CURRENT 0
#define PacketID_RPM 0
#define PacketID_POS 0
#define PacketID_ORIGIN 0

    uint8_t ctrl_id;

    AK_Motor(uint8_t ctrl_id_):
        ctrl_id(ctrl_id_) {}

    void SetCurrent(float current) {
        auto buf = buffer_append_int32(current * 1000.0f);
        auto id  = ctrl_id | (PacketID_CURRENT << 8);
        auto len = 4;
    }

    void SetVelocity(uint8_t ctrl_id, float rpm) {
        auto buf = buffer_append_int32(rpm);
        auto id  = ctrl_id | (PacketID_RPM << 8);
        auto len = 4;
    }

    void SetPosition(uint8_t ctrl_id, float pos) {
        auto buf = buffer_append_int32(pos * 10000.0f);
        auto id  = ctrl_id | (PacketID_POS << 8);
        auto len = 4;
    }

    void SetOrigin(uint8_t ctrl_id, uint8_t set_origin_mode) {
        auto buf = set_origin_mode;
        auto id  = ctrl_id | (PacketID_ORIGIN << 8);
        auto len = 1;
    }

    std::array<uint8_t, 4> buffer_append_int32(int32_t number) {
        std::array<uint8_t, 4> buf;
        buf[0] = number >> 24;
        buf[1] = number >> 16;
        buf[2] = number >> 8;
        buf[3] = number;
        return buf;
    }
};

constexpr AK_Motor::Param AK10_9_PARAM = { 12.5f, 50.0f, 0, 500.0f, 0, 5.0f, 65.0f },
                          AK60_6_PARAM = { 12.5f, 45.0f, 0, 500.0f, 0, 5.0f, 15.0f };

} // namespace comm
} // namespace vlib

#endif

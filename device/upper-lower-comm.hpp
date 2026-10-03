/**
 * @file upper-lower-comm.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-24
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef UPPER_LOWER_COMM_HPP
#define UPPER_LOWER_COMM_HPP

#define PITCH 0
#define YAW 1

#include "stdbool.h"
#include "stdint.h"

#pragma pack(push, 1)

namespace comm {

// 自瞄
class Aimbot {
public:
    enum class Protocol {
        Debug,
        FYT2024_USB,
        SP2025_USB,
        SP2025_CAN,
    } protocol;
    enum class Mode {
        IDLE,       // 空闲
        AUTO_AIM,   // 自瞄qqq
        SMALL_BUFF, // 小符
        BIG_BUFF    // 大符
    };
    enum class Color {
        Blue,
        Red,
    };

    // sp2026 can
    static constexpr uint32_t AutoAim_TxID = 0x110;
    static constexpr uint32_t AutoAim_RxID = 0xFF;

    Aimbot(Protocol protocol, void (*tx_func)(uint8_t* buf, uint64_t len)):
        protocol(protocol),
        tx_func(tx_func) {}

    /* ---------------- upper interface ---------------- */

    inline float Get_PitchError(void) const {
        return vision.Error[0];
    }
    inline float Get_YawError(void) const {
        return vision.Error[1];
    }
    inline bool Get_FireFlag(void) const {
        return vision.Cmd[0] == (uint8_t)Control_SP2025_USB::Mode::Fire;
    }
    bool debug11;
    inline bool Get_LockedFlag(void) {
        debug11 = vision.Cmd[0] != 0;
        return debug11;
    }

    inline void Set_Mode(Mode mode) {
        vision.Mode = mode;
    }
    inline void Set_Angle(float yaw, float pitch, float roll) {
        vision.yaw = yaw;
        vision.pitch = pitch;
        vision.roll = roll;
    }
    inline void SetQuat(float q0, float q1, float q2, float q3) {
        sp_feedback.quat[0] = q0;
        sp_feedback.quat[1] = q1;
        sp_feedback.quat[2] = q2;
        sp_feedback.quat[3] = q3;
    }

    inline void Set_BulletVelocity(float velocity) {
        vision.Velocity = velocity;
    }

    // void Set_Color(Color color);

    /* ---------------- lower interface ---------------- */

    void Receive_USB(uint8_t* buf);
    void Receive_CAN(uint8_t* buf);
    void Transmit(void);

    void Monitor(void);
    inline const bool& Is_Online(void) {
        return is_online;
    }

private:
    void (*tx_func)(uint8_t* buf, uint64_t len);

    uint8_t heartbeat_cnt;

    bool is_online;

    struct Vision {
        uint8_t Vision_RobotID;
        Mode Mode;
        float Velocity;

        uint8_t Cmd[2];
        double Error[2];
        float Distance;
        uint8_t Data_Error;

        float yaw, pitch, roll;
    } vision;

    /* --------------------------------- FYT 2024 -------------------------------- */

    /// @brief 中南s2024自瞄通信 - USB CDC
    void Feedback_Encode_FYT2024_USB(void);
    /// @brief 中南s2024自瞄通信 - USB CDC
    void Control_Decode_FYT2024_USB(uint8_t* Buf);

    /* --------------------------------- SP 2025 -------------------------------- */

    uint64_t debug_cnt = 0;

    __attribute__((packed)) struct Feedback_SP2025_USB {
        uint8_t head[2];
        uint8_t mode; // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
        float quat[4];
        float bullet_speed;
        uint16_t crc16;
    } sp_feedback;

    __attribute__((packed)) struct Control_SP2025_USB {
        uint8_t head[2];
        enum class Mode : uint8_t {
            Idle = 0,   // 空闲
            Locked = 1, // 锁定
            Fire = 2,   // 发射
        } mode;
        float yaw_err;
        float pitch_err;
        uint16_t crc16;
    } sp_control;

    /// @brief 同济s2025自瞄 - USB CDC
    void Feedback_Encode_SP2025_USB(void);
    /// @brief 同济s2025自瞄 - USB CDC
    void Control_Decode_SP2025_USB(uint8_t* buf);

    /// @brief 同济s2025自瞄 - CAN
    void Feedback_Encode_SP2025_CAN(void);
    /// @brief 同济s2025自瞄 - CAN
    void Control_Decode_SP2025_CAN(const uint8_t* buf);

    /* -------------------------------- utils -------------------------------- */

    void Float_to_Byte(float f, unsigned char byte[]);
    int16_t uint8_Int16(uint8_t Data1, uint8_t Data2);
    float uint8_Float(uint8_t Data1, uint8_t Data2);
};

} // namespace comm

#pragma pack(pop)

#endif

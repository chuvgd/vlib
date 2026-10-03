/**
 * @file upper-lower-comm.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-04-24
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "stdint.h"
#include "string.h"

#include "upper-lower-comm.hpp"

#include "utils/crc.hpp"

#define Red 0x00
#define Blue 0x01

#define AngToRad(x) (float)(x) * (3.1415926f) / 180.f
#define RadToAng(x) (double)(x) * 180 / (3.1415926)

namespace comm {

/* ---------------------------------------------------------------- public ---------------------------------------------------------------- */

void Aimbot::Receive_USB(uint8_t* buf) {
    heartbeat_cnt = 0;
    is_online = true;

    switch (protocol) {
        case Protocol::FYT2024_USB:
            Control_Decode_FYT2024_USB(buf);
            break;

        case Protocol::SP2025_USB:
            Control_Decode_SP2025_USB(buf);
            break;

        default:
            break;
    }
}

void Aimbot::Receive_CAN(uint8_t* buf) {
    heartbeat_cnt = 0;
    is_online = true;

    switch (protocol) {
        case Protocol::SP2025_CAN:
            Control_Decode_SP2025_CAN(buf);
            break;

        default:
            break;
    }
}

void Aimbot::Transmit(void) {
    switch (protocol) {
        case Protocol::FYT2024_USB:
            Feedback_Encode_FYT2024_USB();
            break;

        case Protocol::SP2025_USB:
            Feedback_Encode_SP2025_USB();
            break;

        case Protocol::SP2025_CAN:
            Feedback_Encode_SP2025_CAN();
            break;

        default:
            break;
    }
}

void Aimbot::Monitor(void) {
    if (heartbeat_cnt < 50) {
        heartbeat_cnt++;
    } else {
        is_online = false;
    }
}

/* ---------------------------------------------------------------- FYT s2024 ---------------------------------------------------------------- */
// 中南s2024自瞄通信 - USB CDC

void Aimbot::Feedback_Encode_FYT2024_USB(void) {
    // uint8_t robo_color = vision.Vision_RobotID >= 10 ? Red : Blue; // 这个没用？

    uint8_t imu[3][4] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

    Float_to_Byte(vision.pitch, imu[0]);
    Float_to_Byte(vision.yaw, imu[1]);
    Float_to_Byte(vision.roll, imu[2]);

    uint8_t buf[16] = { 0xff,      0x00,      imu[2][0], imu[2][1], imu[2][2], imu[2][3],
                        imu[0][0], imu[0][1], imu[0][2], imu[0][3], imu[1][0], imu[1][1],
                        imu[1][2], imu[1][3], 0x00,      0x0d };
    tx_func(buf, sizeof(buf));
}

void Aimbot::Control_Decode_FYT2024_USB(uint8_t* Buf) {
    union {
        float data_raw;
        uint8_t data[4];
    } u8_f32[3] = { 0, 0, 0 };

    // verify frame header
    if (Buf[0] == 0xff && Buf[15] == 0x0d) {
        // ok
    } else
        return;

    // unknow?
    vision.Cmd[1] = Buf[1];

    // yaw
    for (int i = 0; i < 4; i++)
        u8_f32[0].data[i] = Buf[i + 2];
    vision.Error[1] = u8_f32[0].data_raw;

    // pitch
    for (int i = 0; i < 4; i++)
        u8_f32[1].data[i] = Buf[i + 6];
    vision.Error[0] = u8_f32[1].data_raw;

    // distance???
    for (int i = 0; i < 4; i++)
        u8_f32[2].data[i] = Buf[i + 10];
    vision.Distance = u8_f32[2].data_raw;
}

/* ---------------------------------------------------------------- SuperPower s2025 ---------------------------------------------------------------- */
// 同济s2025自瞄

/* -------------------------------- USB CDC -------------------------------- */

void Aimbot::Feedback_Encode_SP2025_USB(void) {
    sp_feedback.head[0] = 'S';
    sp_feedback.head[1] = 'P';

    sp_feedback.mode = (uint8_t)vision.Mode;
    sp_feedback.bullet_speed = vision.Velocity;

    constexpr uint8_t len = sizeof(sp_feedback);
    sp_feedback.crc16 =
        utils::Get_CRC16_Check_Sum((uint8_t*)&sp_feedback, len - sizeof(sp_feedback.crc16), 0xffff);
    tx_func((uint8_t*)&sp_feedback, len);
}

void Aimbot::Control_Decode_SP2025_USB(uint8_t* buf) {
    uint8_t len = sizeof(sp_control);
    if (buf[0] == 'S' && buf[1] == 'P') {
        if (utils::Verify_CRC16_Check_Sum(buf, len) == 0x01) {
            memcpy(&sp_control, buf, len);
            vision.Data_Error = 0;
        } else
            vision.Data_Error = 1;
    } else
        vision.Data_Error = 1;

    vision.Cmd[0] = (uint8_t)sp_control.mode;

    // 此处加了个低通
    // vision.Error[0] = Low_Pass(RadToAng(data->pitch_err), 0.05);
    // vision.Error[1] = Low_Pass(RadToAng(data->yaw_err), 0.05);
    vision.Error[0] = (double)sp_control.pitch_err;
    vision.Error[1] = (double)sp_control.yaw_err;

    debug_cnt++;
}

/* -------------------------------- CAN -------------------------------- */

void Aimbot::Feedback_Encode_SP2025_CAN(void) {
    // ID = AutoAim_TxID;
    uint8_t buf[8] = { 0 };

    int16_t data[4] = {
        (int16_t)(vision.Velocity * 100),
        uint8_Int16(0, 0),
        0,
        0,
    };

    for (uint8_t i = 0; i < 4; i++)
        buf[i] = data[i];

    tx_func(buf, 0);
}

void Aimbot::Control_Decode_SP2025_CAN(const uint8_t* buf) {
    constexpr float K = 0.0001;
    vision.Cmd[0] = buf[0];
    vision.Cmd[1] = buf[1];
    vision.Error[0] = uint8_Float(buf[4], buf[5]) * K; // pitch
    vision.Error[1] = uint8_Float(buf[2], buf[3]) * K; // yaw
    vision.Distance = uint8_Float(buf[6], buf[7]) * K;
}

/* ---------------------------------------------------------------- utils ---------------------------------------------------------------- */

void Aimbot::Float_to_Byte(float f, unsigned char byte[]) {
    union FloatLongType {
        float fdata;
        unsigned long ldata;
    } fl { .fdata = f };

    byte[0] = (unsigned char)fl.ldata;
    byte[1] = (unsigned char)(fl.ldata >> 8);
    byte[2] = (unsigned char)(fl.ldata >> 16);
    byte[3] = (unsigned char)(fl.ldata >> 24);
}
int16_t Aimbot::uint8_Int16(uint8_t Data1, uint8_t Data2) {
    return (int16_t)((uint16_t)((uint8_t)Data1 << 8) | (uint8_t)Data2);
}

float Aimbot::uint8_Float(uint8_t Data1, uint8_t Data2) {
    return (float)uint8_Int16(Data1, Data2);
}

} // namespace comm

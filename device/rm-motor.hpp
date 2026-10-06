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

#ifndef RM_MOTOR_HPP
#define RM_MOTOR_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace comm {

template<size_t PORT>
class RM_Motor {
    static std::array<uint8_t, 8> ctrl_msg_buf;

public:
    uint8_t id;

    int16_t encoder;      // 2pi/8191
    int16_t velocity;     // rpm
    int16_t current_fdb;  // 20A/16384
    uint8_t temperature;  // C
    int16_t current_ctrl; // 20A/16384

    void Encode(Control data) {
        ctrl_msg_buf[0] = data[0] >> 8;
        ctrl_msg_buf[1] = data[0];
        ctrl_msg_buf[2] = data[1] >> 8;
        ctrl_msg_buf[3] = data[1];
        ctrl_msg_buf[4] = data[2] >> 8;
        ctrl_msg_buf[5] = data[2];
        ctrl_msg_buf[6] = data[3] >> 8;
        ctrl_msg_buf[7] = data[3];
    }

    void Decode(uint8_t* buf) {
        encoder = (int16_t)((buf[0] << 8) | buf[1]);
        velocity = (int16_t)((buf[2] << 8) | buf[3]);
        current_fdb = (int16_t)((buf[4] << 8) | buf[5]);
        temperature = buf[6];
    }

    void Setpoint(int16_t current) {
        current_ctrl = current;
        ctrl_msg_buf[((id - 1) % 4) * 2] = current_ctrl >> 8;
        ctrl_msg_buf[((id - 1) % 4) * 2 + 1] = current_ctrl;
    }
};

} // namespace comm

#define M3508_TX_ID_1 0x200
#define M3508_TX_ID_2 0x1FF
#define M3508_RX_ID(id) (0x200 + (id))

#define M2006_TX_ID_1 0x200
#define M2006_TX_ID_2 0x1FF
#define M2006_RX_ID(id) (0x200 + (id))

#define GM6020_TX_V_ID_1 0x1FF
#define GM6020_TX_V_ID_2 0x2FF
#define GM6020_TX_A_ID_1 0x1FE
#define GM6020_TX_A_ID_2 0x2FE
#define GM6020_RX_ID(id) (0x204 + (id))

#define M3508_GEAR_RATIO (3591.f / 187.f) // M3508 电机原版减速箱的减速比
#define M2006_GEAR_RATIO (36 / 1)

// 电机读取数据

#define RM_MOTOR_ANGLE(encoder) ((float)encoder * 2.0 * PI / 8191.0)     // unit: rad
#define RM_MOTOR_VELOCITY(self) ((float)(self)->velocity / 60 * 2 * PI)  // unit: rad/s
#define RM_MOTOR_CURRENT(self) ((float)(self)->current_fdb * 20 / 16384) // unit: A
#define RM_MOTOR_TEMP(self) ((float)(self)->temperature)                 // unit: C

// 关于 HEXROLL 减速箱 + M3508 电机下的一些转换

#define HEXROLL_GEAR_RATIO (268.f / 17.f) // HEXROLL 减速比

#define HEXROLL_VELOCITY(self) \
    ((float)(self)->velocity / 60.0f * 2 * PI / HEXROLL_GEAR_RATIO) // unit: rad/s

#define HEXROLL_TORQUE_TO_CURRENT(torque) (int16_t)(torque * 3326.263229308005)
#define HEXROLL_CURRENT_TO_TORQUE(current) (int16_t)(current / 3326.263229308005)
// approx torque * 3326.263229308005
// #define HEXROLL_TORQUE_TO_CURRENT(torque) \
// 	(int16_t)(torque / HEXROLL_GEAR_RATIO * M3508_GEAR_RATIO / 0.3 / 20 \ * 16384)

#endif

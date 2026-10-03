/**
 * @file dm-motor.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 达妙电机
 * @version 0.1.0
 * @date 2026-05-08
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "dm-motor.hpp"
#include <algorithm>

namespace rb2 {
namespace device {

DM_Motor::DM_Motor(): param(dm4310) {}

/* ---------------------------------------------------------------- command ---------------------------------------------------------------- */

void DM_Motor::SetCommand(Command cmd) {
    std::fill_n(buf, 7, 0xFF);
    buf[7] = static_cast<uint8_t>(cmd);
}

/* ---------------------------------------------------------------- feedback ---------------------------------------------------------------- */

void DM_Motor::Receive(uint8_t* buf) {}

/**
 * @brief 
 * 
 * @param buf 
 * @return true 
 * @return false 
*/
void DM_Motor::Feedback_Decode(uint8_t* buf) {
    feedback.id = buf[0] & 0x0F;
    feedback.state = (State)(buf[0] >> 4);

    int position_int = (buf[1] << 8) | buf[2];
    int velocity_int = (buf[3] << 4) | (buf[4] >> 4);
    int torque_int = ((buf[4] & 0xF) << 8) | buf[5];

    feedback.temperature_mos = (float)(buf[6]);
    feedback.temperature_rotor = (float)(buf[7]);

    feedback.position = uint_to_float(position_int, param.p_min, param.p_max, 16);
    feedback.velocity = uint_to_float(velocity_int, param.v_min, param.v_max, 12);
    feedback.torque = uint_to_float(torque_int, param.t_min, param.t_max, 12);
}

/* ---------------------------------------------------------------- control ---------------------------------------------------------------- */

void DM_Motor::Transmit(Control data) {}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void DM_Motor::MIT_Encode(float kp, float kd, float position, float velocity, float torque) {
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = float_to_uint(position, param.p_min, param.p_max, 16);
    vel_tmp = float_to_uint(velocity, param.v_min, param.v_max, 12);
    kp_tmp = float_to_uint(kp, param.kp_min, param.kp_max, 12);
    kd_tmp = float_to_uint(kd, param.kd_min, param.kd_max, 12);
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

/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   vel:			速度给定值
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void DM_Motor::PosSpd_Encode(float position, float velocity) {
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    pbuf = (uint8_t*)&position;
    vbuf = (uint8_t*)&velocity;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);
}
/**
************************************************************************
* @brief:      	speed_ctrl: 速度控制函数
* @param[in]:   vel: 			速度给定值
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void DM_Motor::Speed_Encode(float velocity) {
    uint8_t* vbuf;
    uint8_t data[8];

    vbuf = (uint8_t*)&velocity;

    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);
}

/* ---------------------------------------------------------------- tool ---------------------------------------------------------------- */

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int DM_Motor::float_to_uint(float x_float, float x_min, float x_max, int bits) {
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float DM_Motor::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

} // namespace device
} // namespace rb2

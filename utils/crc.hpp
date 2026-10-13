/**
 * @file crc.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief CRC校验
 * @version 0.1.0
 * @date 2026-10-13
 *
 * @copyright Copyright (c) Serialist 2026
 *
 * @note
 * CRC校验是用在和裁判系统的通信，云台和裁判系统交互的通道是图传链路
 * 底盘和裁判系统交互的通道是电源管理模块
 */

#ifndef VLIB_UTILS_CRC_HPP
#define VLIB_UTILS_CRC_HPP

#include <stdint.h>

namespace vlib {

// crc8
uint8_t  Get_CRC8_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint32_t Verify_CRC8_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
void     Append_CRC8_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);

// crc16
uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
void     Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);

} // namespace vlib

#endif // VLIB_UTILS_CRC_HPP

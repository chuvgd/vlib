/**
 * @file convert.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-14
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef CONVERT_HPP
#define CONVERT_HPP

namespace rb2 {
namespace convert {

float Bit2Float(int x_int, float x_min, float x_max, int Bits);
int Float2Bit(float x, float x_min, float x_max, int bits);

} // namespace convert
} // namespace rb2

#endif

/**
 * @file convert.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-14
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#include "convert.hpp"

namespace rb2 {
namespace convert {

float Bit2Float(int x_int, float x_min, float x_max, int Bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << Bits) - 1)) + offset;
}

int Float2Bit(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

} // namespace convert
} // namespace rb2

/**
 * @file signal.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 信号发生器（嵌入式专用，无异常、无动态内存）
 * @version 0.1.0
 * @date 2026-07-17
 *
 * @copyright Copyright (c) Serialist 2026
 *
*/

#ifndef SIGNAL_HPP
#define SIGNAL_HPP

#include <cmath>
#include <cstdint>

namespace algorithm {

template<typename T>
class Generate_Sin {
private:
    T AMPLITUDE = 1;
    T FREQUENCY = 1;
    T PHASE_OFFSET = 0;
    T BIAS = 0;

public:
    Generate_Sin(T AMPLITUDE, T FREQUENCY, T PHASE_OFFSET, T BIAS):
        AMPLITUDE(AMPLITUDE),
        FREQUENCY(FREQUENCY),
        PHASE_OFFSET(PHASE_OFFSET),
        BIAS(BIAS) {};
    T Get(float time) {
        return AMPLITUDE * std::sin(FREQUENCY * time + PHASE_OFFSET) + BIAS;
    }
};

template<typename T>
class Generate_Square {
private:
    T AMPLITUDE = 1;
    T FREQUENCY = 1;
    T PHASE_OFFSET = 0;
    T BIAS = 0;

public:
    Generate_Square(T AMPLITUDE, T FREQUENCY, T PHASE_OFFSET, T BIAS):
        AMPLITUDE(AMPLITUDE),
        FREQUENCY(FREQUENCY),
        PHASE_OFFSET(PHASE_OFFSET),
        BIAS(BIAS) {};
    T Get(float time) {
        return AMPLITUDE * std::signbit(std::sin(FREQUENCY * time + PHASE_OFFSET)) + BIAS;
    }
};

} // namespace algorithm

#endif

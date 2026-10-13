/**
 * @file mathtool.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-05-10
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef VLIB_ALGO_MATHTOOL_HPP
#define VLIB_ALGO_MATHTOOL_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

namespace vlib {
namespace algo {

/* ================================================================ detail ================================================================ */

namespace detail {

// 平台适配：确保 float / double / long double 使用各自精度的 floor / fabs，
// 避免在部分嵌入式标准库中隐式提升为 double。
inline float Floor(float x) {
    return std::floor(x);
}
inline double Floor(double x) {
    return std::floor(x);
}
inline long double Floor(long double x) {
    return std::floor(x);
}

inline float Abs(float x) {
    return std::abs(x);
}
inline double Abs(double x) {
    return std::abs(x);
}
inline long double Abs(long double x) {
    return std::abs(x);
}

} // namespace detail

/* ================================================================ constant ================================================================ */

constexpr float pi      = 3.14159265358979323846; // pi
constexpr float half_pi = 1.570796326794897;      // pi/2
constexpr float two_pi  = 6.283185307179586;      // 2pi
constexpr float e       = 2.71828182845904523536; // euler number

/* ================================================================ function ================================================================ */

/// @brief 弧度转角度
template<typename T>
constexpr T Rad2Deg(T rad) {
    return rad * T(57.295779513);
}

/// @brief 角度转弧度
template<typename T>
constexpr T Deg2Rad(T deg) {
    return deg * T(0.01745329252);
}

template<typename T>
constexpr T Sign(T x) {
    return x > T(0) ? T(1) : (x < T(0) ? T(-1) : T(0));
}

/**
 * @brief 快速幂算法 fast power
 *
 * @param a base
 * @param b exponent
 * @return long long
 *
 * @note a ^ b
 */
inline long long FPow(long long a, long long b) {
    long long res = 1;

    while (b) {
        if (b & 1)
            res = res * a;
        a = a * a;
        b >>= 1;
    }
    return res;
}

/**
 * @brief 快速幂取模算法 fast power with mod
 *
 * @param a base
 * @param b exponent
 * @param p mod
 * @return long long
 *
 * @note a ^ b % p
 */
inline long long FPowMod(long long a, long long b, long long p) {
    long long res = 1;

    while (b) {
        if (b & 1)
            res = res * a % p;
        a = a * a % p;
        b >>= 1;
    }
    return res;
}

/// @brief 快速平方根倒数，保留 float，位技巧依赖 float
inline float FinvSqrt(float x) {
    static_assert(sizeof(float) == sizeof(uint32_t), "float must be 4 bytes");

    float    halfnum = 0.5f * x;
    float    y       = x;
    uint32_t i;

    std::memcpy(&i, &y, sizeof(i));
    i = 0x5f375a86u - (i >> 1);
    std::memcpy(&y, &i, sizeof(y));

    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/// @brief 最大公约数 greatest common divisor
inline long long FGcd(long long a, long long b) {
    if (b == 0)
        return a;
    return FGcd(b, a % b);
}

/// @brief 限幅，指针版
template<typename T>
void ClampPtr(T* in, T min, T max) {
    if (*in < min) {
        *in = min;
    } else if (*in > max) {
        *in = max;
    }
}

/// @brief 限幅
template<typename T>
constexpr T Clamp(T value, T min, T max) {
    return std::min(std::max(value, min), max);
}

/// @brief 绝对值限幅
template<typename T>
constexpr T ClampABS(T value, T max) {
    return std::min(std::max(value, -max), max);
}

/// @brief 步进限幅
/// @param prev 前
/// @param value 现在
/// @param min 小
/// @param max 大
template<typename T>
constexpr T StepClamp(T prev, T value, T min, T max) {
    return prev + Clamp(value - prev, min, max);
}

template<typename T>
T Mod(T value, T range) {
    static_assert(std::is_floating_point<T>::value, "Mod requires floating point type");

    if (range == T(0))
        return std::numeric_limits<T>::quiet_NaN();

    return value - detail::Floor(value / range) * range;
}

/**
 * @brief 循环限幅函数
 *
 * @param value
 * @param min
 * @param max
 * @return T
 *
 * @note 不做检查，min > max 未定义
 */
template<typename T>
T LoopClamp(T value, T min, T max) {
    static_assert(std::is_floating_point<T>::value, "LoopClamp requires floating point type");

    T range = max - min;
    return value - detail::Floor((value - min) / range) * range;
}

template<typename T>
T CircleClamp(T value) {
    static_assert(std::is_floating_point<T>::value, "CircleClamp requires floating point type");

    const T two_pi_t = static_cast<T>(two_pi);
    return value - detail::Floor(value / two_pi_t) * two_pi_t;
}

/// @brief 值映射
template<typename T>
constexpr T Remap(T a, T inmin, T inmax, T outmin, T outmax) {
    return outmin + (a - inmin) * (outmax - outmin) / (inmax - inmin);
}

/// @brief 斜坡函数
template<typename T>
constexpr T Ramp(T prev_x, T x, T k_min, T k_max, T dt) {
    return x + Clamp(prev_x - x, k_min * dt, k_max * dt);
}

/**
 * @brief 死区函数
 *
 * @param value 输入
 * @param point 死区点
 * @param deadzone 死区大小
 */
template<typename T>
constexpr T Deadzone(T value, T point, T deadzone) {
    if ((point - deadzone) < value && value < (point + deadzone)) {
        return point;
    } else {
        return value;
    }
}

/// @brief 循环最近距离
/// @param point 当前点（应在 [min, max]）
/// @param setpoint 目标点（应在 [min, max]）
/// @return 循环最近距离
template<typename T>
T LoopNearestDistance(T point, T setpoint, T min, T max) {
    T drct  = setpoint - point;
    T range = max - min;

    if (detail::Abs(drct) <= (range * T(0.5)))
        return drct;
    else if (drct > T(0))
        return drct - range;
    else
        return drct + range;
}

/// @brief 循环最近点
/// @param point 当前点（应在 [min, max]）
/// @param setpoint 目标点（应在 [min, max]）
/// @return 循环最近目标点
template<typename T>
T LoopNearestPoint(T point, T setpoint, T min, T max) {
    return point + LoopNearestDistance(point, setpoint, min, max);
}

/// @brief 圆圈最近距离
/// @param point 当前点（应在 [0, 2PI]）
/// @param setpoint 目标点（应在 [0, 2PI]）
/// @return 套圈最近距离
template<typename T>
T CircleNearestDistance(T point, T setpoint) {
    const T pi_t     = static_cast<T>(pi);
    const T two_pi_t = static_cast<T>(two_pi);

    T drct = setpoint - point;

    if (detail::Abs(drct) <= pi_t)
        return drct;
    else if (drct > T(0))
        return drct - two_pi_t;
    else
        return drct + two_pi_t;
}

/// @brief 圆圈最近点
/// @param point 当前点（应在 [0, 2PI]）
/// @param setpoint 目标点（应在 [0, 2PI]）
/// @return 套圈最近目标点
/// @note 可用于旋转时解算目标点
template<typename T>
T CircleNearestPoint(T point, T setpoint) {
    return point + CircleNearestDistance(point, setpoint);
}

/// @brief 半圈最近距离
/// @param point 当前点（应在 [0, PI]）
/// @param setpoint 目标点（应在 [0, PI]）
/// @return 套半圈最近距离
template<typename T>
T HalfCircleNearestDistance(T point, T setpoint) {
    const T pi_t = static_cast<T>(pi);

    T drct = setpoint - point;

    if (detail::Abs(drct) <= (pi_t * T(0.5)))
        return drct;
    else if (drct > T(0))
        return drct - pi_t;
    else
        return drct + pi_t;
}

/// @brief 半圈最近点
/// @param point 当前点（应在 [0, PI]）
/// @param setpoint 目标点（应在 [0, PI]）
/// @return 套半圈最近目标点
/// @note 可用于旋转时解算目标点
template<typename T>
T HalfCircleNearestPoint(T point, T setpoint) {
    return point + HalfCircleNearestDistance(point, setpoint);
}

} // namespace algo
} // namespace vlib

#endif

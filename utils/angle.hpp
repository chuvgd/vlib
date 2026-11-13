/**
 * @file angle.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 角度值 / 弧度制的转换、计算与表示工具。
 *        Header-only，无 STL 依赖，C++14 起可用。
 *        面向 STM32F103 (Cortex-M3, 无 FPU) 及更高平台。
 * @version 0.3.0
 * @date 2026-10-13
 *
 * @copyright Copyright (c) Serialist 2026
 */

#pragma once

namespace vlib {

/**
 * @brief 编译期 PI（C++14 变量模板）
 */
template<typename Tp>
constexpr Tp PI = static_cast<Tp>(3.141592653589793238462643383279502884L);

/**
 * @brief 角度值 / 弧度制工具类
 */
class Angle {
public:
    /// @brief 默认构造为 0 弧度
    constexpr Angle() noexcept:
        value(0.f) {}

    /// @brief 从弧度显式构造
    explicit constexpr Angle(float rad) noexcept:
        value(rad) {}

    // ---- 取值 ----
    constexpr float rad() const noexcept { return value; }
    constexpr float deg() const noexcept { return value * (180.f / PI<float>); }

    // ---- 二元运算 ----
    // clang-format off
           constexpr Angle operator-(                              ) const noexcept { return Angle(-value          ); }
           constexpr Angle operator+(const Angle& o                ) const noexcept { return Angle( value + o.value); }
           constexpr Angle operator-(const Angle& o                ) const noexcept { return Angle( value - o.value); }
           constexpr Angle operator*(      float  s                ) const noexcept { return Angle( value * s      ); }
           constexpr Angle operator/(      float  s                ) const noexcept { return Angle( value / s      ); }
	friend constexpr Angle operator*(      float  s, const Angle& a)       noexcept { return a * s; }
    // clang-format on

    // ---- 复合赋值（C++14 起可为 constexpr）----
    constexpr Angle& operator+=(const Angle& o) noexcept {
        value += o.value;
        return *this;
    }
    constexpr Angle& operator-=(const Angle& o) noexcept {
        value -= o.value;
        return *this;
    }
    constexpr Angle& operator*=(float s) noexcept {
        value *= s;
        return *this;
    }
    constexpr Angle& operator/=(float s) noexcept {
        value /= s;
        return *this;
    }

    // ---- 比较（浮点 == 请谨慎使用）----
    // clang-format off
    constexpr bool operator==(const Angle& o) const noexcept { return value == o.value; }
    constexpr bool operator!=(const Angle& o) const noexcept { return value != o.value; }
    constexpr bool operator< (const Angle& o) const noexcept { return value <  o.value; }
    constexpr bool operator<=(const Angle& o) const noexcept { return value <= o.value; }
    constexpr bool operator> (const Angle& o) const noexcept { return value >  o.value; }
    constexpr bool operator>=(const Angle& o) const noexcept { return value >= o.value; }
    // clang-format on

    // ---- 工厂 ----
    static constexpr Angle Degree(float d) noexcept { return Angle(d * (PI<float> / 180.f)); }
    static constexpr Angle Radian(float r) noexcept { return Angle(r); }

private:
    float value; // 内部统一以弧度存储
};

// ---- 用户自定义字面量（C++11 起可用）----

namespace literals {

// clang-format off
constexpr Angle operator""_deg(long double        d) noexcept { return Angle::Degree(static_cast<float>(d)); }
constexpr Angle operator""_deg(unsigned long long d) noexcept { return Angle::Degree(static_cast<float>(d)); }
constexpr Angle operator""_rad(long double        r) noexcept { return         Angle(static_cast<float>(r)); }
constexpr Angle operator""_rad(unsigned long long r) noexcept { return         Angle(static_cast<float>(r)); }
// clang-format on

} // namespace literals

} // namespace vlib

/**
 * @file simple-filter.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-01
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
 */

#ifndef SIMPLE_FILTER_HPP
#define SIMPLE_FILTER_HPP

#include <cstdint>

namespace vlib {
namespace algo {

/* ---------------------------------------------------------------- LowPass_Order_1 ---------------------------------------------------------------- */

class LowPass_Order_1 {
public:
    /// @param cutoff_frequency 截止频率
    /// @param sample_rate 采样率
    LowPass_Order_1(float cutoff_frequency, float sample_rate);

    /// @param alpha 滤波系数
    LowPass_Order_1(float alpha);

    /// @param input 输入
    /// @return 滤波结果
    float& update(float input);

private:
    float cutoff_frequency;
    float sample_rate;
    float alpha;
    float output;
};

/* ---------------------------------------------------------------- ZeroOrder_Holder ---------------------------------------------------------------- */

// 零阶保持器
class ZeroOrder_Holder {
public:
    ZeroOrder_Holder(void);

    float& update(float input);

private:
    float prev_value;
};

/* ---------------------------------------------------------------- Moving_Average_Filter ---------------------------------------------------------------- */

/// @brief 滑动均值滤波器
///
/// @note
/// - 使用循环窗口 + 增量更新（O(1)），周期性地整窗重算以抑制浮点累积误差。
/// - 若输入出现 inf / nan，请检查数据范围（尤其是填充窗口的初值）。
class Moving_Average_Filter {
public:
    static constexpr uint8_t WINDOW_MAX_SIZE  = 32;
    static constexpr uint32_t RECALC_INTERVAL = 79;

    explicit Moving_Average_Filter(uint8_t width);

    /// @brief 更新一次并返回当前均值
    /// @param input 新样本
    float& update(float input);

    float value(void) const {
        return average;
    }
    uint8_t getWidth(void) const {
        return width;
    }

private:
    float window[WINDOW_MAX_SIZE];
    float average;
    uint8_t width;
    uint8_t index;
    uint32_t recalc_cnt;
};

} // namespace algo
} // namespace vlib

#endif

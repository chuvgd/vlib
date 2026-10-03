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

namespace rb2 {
namespace algorithm {

class LowPass_Order_1 {
public:
    /// @brief constructor
    /// @param cutoff_frequency 截止频率
    /// @param sample_rate 采样率
    LowPass_Order_1(float cutoff_frequency, float sample_rate);

    /// @brief constructor
    /// @param alpha 滤波系数
    LowPass_Order_1(float alpha);

    /// @brief 更新
    /// @param input 输入
    /// @return 滤波结果
    float& update(float input);

private:
    float cutoff_frequency;
    float sample_rate;
    float alpha;
    float output;
};

// 零阶保持器
class ZeroOrder_Holder {
public:
    ZeroOrder_Holder(void);

    float& update(float input);

private:
    float prev_value;
};

} // namespace algorithm
} // namespace rb2

#endif // SIMPLE_FILTER_HPP

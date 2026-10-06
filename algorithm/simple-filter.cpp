/**
 * @file simple-filter.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-01
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
 */

#include "simple-filter.hpp"

namespace vlib {
namespace algo {

/* ---------------------------------------------------------------- LowPass_Order_1 ---------------------------------------------------------------- */

LowPass_Order_1::LowPass_Order_1(float cutoff_frequency, float sample_rate) {
    this->cutoff_frequency = cutoff_frequency;
    this->sample_rate      = sample_rate;

    alpha = 2.0f * cutoff_frequency / sample_rate;

    output = 0;
}

LowPass_Order_1::LowPass_Order_1(float alpha) {
    this->alpha = alpha;

    cutoff_frequency = 0;
    sample_rate      = 0;
    output           = 0;
}

float& LowPass_Order_1::update(float input) {
    output = alpha * input + (1.0f - alpha) * output;
    return output;
}

/* ---------------------------------------------------------------- ZeroOrder_Holder ---------------------------------------------------------------- */

ZeroOrder_Holder::ZeroOrder_Holder(void) {
    prev_value = 0;
}

float& ZeroOrder_Holder::update(float input) {
    if (input != 0)
        prev_value = input;
    return prev_value;
}

/* ---------------------------------------------------------------- Moving_Average_Filter ---------------------------------------------------------------- */

Moving_Average_Filter::Moving_Average_Filter(uint8_t width):
    average(0.0f),
    width(width),
    index(0),
    recalc_cnt(0) {
    if (width == 0 || width > WINDOW_MAX_SIZE) {
        // 非法宽度则退化为宽度 1（直通），避免 UB
        this->width = 1;
    }
    for (uint8_t i = 0; i < WINDOW_MAX_SIZE; i++) {
        window[i] = 0.0f;
    }
}

float& Moving_Average_Filter::update(float input) {
    const float old = window[index];
    window[index]   = input;
    index           = static_cast<uint8_t>((index + 1) % width);

    if (recalc_cnt % RECALC_INTERVAL) {
        // 增量更新：用新值替换旧值对均值的贡献
        average += (input - old) / static_cast<float>(width);
    } else {
        // 周期性整窗重算，抑制浮点累计误差
        average = 0.0f;
        for (uint8_t i = 0; i < width; i++) {
            average += window[i] / static_cast<float>(width);
        }
    }

    recalc_cnt++;
    return average;
}

} // namespace algo
} // namespace vlib

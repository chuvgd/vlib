/**
 * @file rls.hpp
 * @brief 港科
 * 
 * @note 
 * 依赖：
 * 上交矩阵库 matrix-arm.hpp
 * freertos
*/

#include <cstdint>

#include "algorithm/matrix-arm.hpp"

#pragma once

using vlib::algo::Matf;
using vlib::algo::Vecf;

namespace vlib {
namespace rls {

template<size_t DIM>
class RLS {
public:
    RLS() = delete;

    constexpr RLS(float delta_, float lambda_):

        lambda(lambda_),
        delta(delta_),
        lastUpdate(0),
        cnt(0),
        default_param(Vecf<DIM>::zeros()) {
        reset();
        validate();
    }

    constexpr RLS(float delta_, float lambda_, Vecf<DIM> initParam):
        RLS(delta_, lambda_) {
        default_param = initParam;
    }

    void Reset() {
        transM = Matf<DIM, DIM>::eye() * delta;
        gain   = Vecf<DIM>::zeros();
        param  = Vecf<DIM>::zeros();
    }

    const Vecf<DIM>& update(Vecf<DIM>& sample, float actualOutput) {
        gain = (transM * sample) / (1 + (sample.trans() * transM * sample)[0][0] / lambda) / lambda;
        param += gain * (actualOutput - (sample.trans() * param)[0][0]);
        transM = (transM - gain * sample.trans() * transM) / lambda;

        cnt++;
        // lastUpdate = xTaskGetTickCount();

        return param;
    }

    void SetParam(const Vecf<DIM>& updatedParams) {
        param = default_param = updatedParams;
    }

    constexpr Vecf<DIM>& GetParam() const {
        return param;
    }

    const float& GetOutput() const {
        return output;
    }

private:
    void validate() const {
        configASSERT(lambda >= 0.0f || lambda <= 1.0f);
        configASSERT(delta > 0);
    }

    void configASSERT(bool expr) {
        error_flag = expr;
    }

    float lambda; // The forget index
    float delta;  // Intialized value of the transferred matrix

    uint32_t cnt; // Total update Count
    bool error_flag;

    Matf<DIM, DIM> transM; // Transfer matrix instance
    Vecf<DIM> gain;        // Gain vector for params update
    Vecf<DIM> param;       // Params vector
    Vecf<DIM> default_param;
    float output; // Estimated / filtered output
};

} // namespace rls
} // namespace vlib

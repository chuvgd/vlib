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

#include "algorithm/arm-matrix.hpp"

#pragma once

namespace vlib {

using ARM::Matrix;
using ARM::Vector;

template<size_t DIM>
class RLS {
public:
    RLS() = delete;

    constexpr RLS(float delta_, float lambda_):

        lambda(lambda_),
        delta(delta_),
        lastUpdate(0),
        cnt(0),
        default_param(Vector<DIM>::zeros()) {
        reset();
        validate();
    }

    constexpr RLS(float delta_, float lambda_, Vector<DIM> initParam):
        RLS(delta_, lambda_) {
        default_param = initParam;
    }

    void Reset() {
        transM = Matrix<DIM, DIM>::eye() * delta;
        gain   = Vector<DIM>::zeros();
        param  = Vector<DIM>::zeros();
    }

    const Vector<DIM>& update(Vector<DIM>& sample, float actualOutput) {
        gain = (transM * sample) / (1 + (sample.trans() * transM * sample)[0][0] / lambda) / lambda;
        param += gain * (actualOutput - (sample.trans() * param)[0][0]);
        transM = (transM - gain * sample.trans() * transM) / lambda;

        cnt++;
        // lastUpdate = xTaskGetTickCount();

        return param;
    }

    void SetParam(const Vector<DIM>& updatedParams) {
        param = default_param = updatedParams;
    }

    constexpr Vector<DIM>& GetParam() const {
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
    bool     error_flag;

    Matrix<DIM, DIM> transM; // Transfer matrix instance
    Vector<DIM>      gain;   // Gain vector for params update
    Vector<DIM>      param;  // Params vector
    Vector<DIM>      default_param;
    float            output; // Estimated / filtered output
};

} // namespace vlib

/**
 * @file kalman-filter.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-07-11
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <cstddef>

#include "algorithm/linear-algebra.hpp"

namespace observer {

using algebra::Matrix;
using algebra::Vector;
using std::size_t;

template<typename T, size_t MEASURE_SIZE, size_t STATE_SIZE>
class Kalman_Filter {
private:
    // 状态向量
    Vector<T, STATE_SIZE> x_prior;     // 先验状态
    Vector<T, STATE_SIZE> x_posterior; // 后验状态
    Vector<T, MEASURE_SIZE> z;         // 观测值

    // 系统矩阵
    Matrix<T, STATE_SIZE, STATE_SIZE> F;   // 状态转移
    Matrix<T, MEASURE_SIZE, STATE_SIZE> H; // 观测映射

    // 卡尔曼增益
    Matrix<T, STATE_SIZE, MEASURE_SIZE> K;

    // 协方差矩阵
    Matrix<T, STATE_SIZE, STATE_SIZE> P_prior;     // 先验协方差
    Matrix<T, STATE_SIZE, STATE_SIZE> P_posterior; // 后验协方差

    // 噪声协方差（修正维度）
    Matrix<T, STATE_SIZE, STATE_SIZE> Q;     // 过程噪声
    Matrix<T, MEASURE_SIZE, MEASURE_SIZE> R; // 观测噪声

public:
    Kalman_Filter(
        const Matrix<T, STATE_SIZE, STATE_SIZE>& F_,
        const Matrix<T, MEASURE_SIZE, STATE_SIZE>& H_,
        const Matrix<T, STATE_SIZE, STATE_SIZE>& Q_,
        const Matrix<T, MEASURE_SIZE, MEASURE_SIZE>& R_,
        const Vector<T, STATE_SIZE>& x0,
        const Matrix<T, STATE_SIZE, STATE_SIZE>& P0
    ):
        x_posterior(x0),
        F(F_),
        H(H_),
        P_posterior(P0),
        Q(Q_),
        R(R_) {}

    // ---------------- 预测 + 更新 ----------------
    const Vector<T, STATE_SIZE>& Step(const Vector<T, MEASURE_SIZE>& z) {
        this->z = z;

        // 预测阶段
        Predict();

        // 更新阶段
        return Update(z);
    }

    // ---------------- 预测阶段：1、2 ----------------
    const Vector<T, STATE_SIZE>& Predict() {
        // 1. 先验状态
        x_prior = F * x_posterior;

        // 2. 先验协方差
        P_prior = F * P_posterior * F.transpose() + Q;

        return x_prior;
    }

    // ---------------- 更新阶段：3、4、5 ----------------
    const Vector<T, STATE_SIZE>& Update(const Vector<T, MEASURE_SIZE>& z) {
        // 3. 卡尔曼增益
        /// @todo the inverse operation need to optimize, and must check singular matrix
        auto H_T = H.transpose();
        K = P_prior * H_T * (H * P_prior * H_T + R).inverse();

        // 4. 后验状态
        x_posterior = x_prior + K * (z - H * x_prior);

        // 5. 后验协方差
        auto I = Matrix<T, STATE_SIZE, STATE_SIZE>::eye();
        // 简化形式，数学上等价，但数值上不保证对称正定，长跑容易让 P 失对称甚至发散
        P_posterior = (I - K * H) * P_prior;
        // 更稳的是 Joseph 形式，但算力需求大
        // auto KH = K * H;
        // P_posterior = (I - KH) * P_prior * (I - KH).transpose() + K * R * K.transpose();

        return x_posterior;
    }
};

} // namespace observer

#endif

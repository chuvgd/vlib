#include "sim-module.hpp"

namespace rb2 {
namespace algorithm {

SecondOrderSystem::SecondOrderSystem(double mass, double damping) {
    this->m = mass;
    this->kf = damping;
}

// 获取当前位置
double SecondOrderSystem::position() {
    return x;
}

// 单步更新（核心：正确二阶系统离散求解）
void SecondOrderSystem::step(double u, double dt) {
    // 二阶系统动力学：加速度 a = (u - kf*v) / m
    double a = (u - kf * v) / m;

    // 欧拉积分（简单稳定）
    v = v + a * dt; // 更新速度
    x = x + v * dt; // 更新位置
}

// 可选：重置状态
void SecondOrderSystem::reset() {
    x = 0.0;
    v = 0.0;
}

} // namespace algorithm
} // namespace rb2

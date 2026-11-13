/**
 * @file omni-chassis.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 四轮 45° 全向底盘：速度解算 + 力解算（全向轮 / 麦轮共用一套公式）
 * @version 0.2.0
 * @date 2026-10-23
 *
 * @copyright Copyright (c) Serialist 2026
 *
 * @note 坐标约定
 *   x 向前、y 向左、wz 逆时针为正；单位 m、rad/s、N、N·m。
 *   轮序 0=左前 FL、1=右前 FR、2=左后 RL、3=右后 RR。
 *
 * @note 四轮驱动方向（"X" 布局，轮面/辊子都按 45°）
 *   0 FL (+x,+y): c = ( 1,-1)/√2
 *   1 FR (+x,-y): c = ( 1, 1)/√2
 *   2 RL (-x,+y): c = ( 1, 1)/√2
 *   3 RR (-x,-y): c = ( 1,-1)/√2
 *   若实车轮子镜像相反，把对应行的 c 取反即可（力/速度公式同时保持自洽）。
 *
 * @note 速度与力是同一个 J 的两侧（虚功原理 / 功率守恒 Σw·τ = v·F）
 *   速度正解：s   = J v
 *   速度逆解：v   = pinv(J) s
 *   力  逆解：F   = Jᵀ τ                     （轮力矩 -> 底盘力旋量）
 *   力  正解：τ   = J pinv(J)ᵀ F = J (JᵀJ)⁻¹ F（底盘力旋量 -> 轮力矩，最小范数解）
 *   所以力和运动学不是两套模型：J 只需要建一次，力那边直接用 J 的转置。
 *   四轮时 J 是 4x3，M = JᵀJ 是 3x3 对角阵，四个式子都有闭式解（本文件即闭式）。
 *
 * @note 四轮是冗余驱动（4 > 3）：Jᵀ 有一维零空间，τ0 ∝ (1,1,-1,-1)
 *   即"前后对拉"的内力，不产生任何底盘力旋量。下面的闭式解天然不含该分量
 *   （最小范数），所以直接下发力矩不会出现轮子互相打架；但若给每个轮子单独
 *   做力矩闭环，闭环误差可能把内力重新引进来。
 */

#pragma once

namespace vlib {

class Omni_Chassis {
    float ab_ {};  // xwith + ywith
    float u_ {};   // gain / r
    float r_ {};   // r / (4 * gain)
    float rab_ {}; // r / (4 * gain * (xwith + ywith))
    float uab_ {}; // gain * (xwith + ywith) / r

public:
    /// @brief 轮系增益 gain
    /// - 麦轮：辊子 45°、轮面沿车体轴，同样前进速度下轮速就是 v/r，取 1
    /// - 全向轮：轮面本身在 45°，前进速度只有 v·cos45 进到轮子，取 1/√2
    /// 两者的 J 结构完全一样，只差这一个系数（等价于差一个 √2 的等效轮半径）。
    static constexpr float kMecanum = float {1};
    static constexpr float kOmni45  = float {0.70710678118654752440};

    struct Wheel {
        float w[4];
    };

    struct Twist {
        float vx, vy, wz;
    };

    struct Wrench {
        float fx, fy, mz;
    };

    Omni_Chassis() = default;

    /// @param wheel_radius 轮半径
    /// @param xwith 轮心到中心的 x 向距离（半轴距）
    /// @param ywith 轮心到中心的 y 向距离（半轮距）
    /// @param gain kMecanum 或 kOmni45
    constexpr Omni_Chassis(float wheel_radius, float xwith, float ywith, float gain = kOmni45):
        ab_(xwith + ywith),
        u_(gain / wheel_radius),
        r_(wheel_radius / (float {4} * gain)),
        rab_(wheel_radius / (float {4} * gain * (xwith + ywith))),
        uab_(gain * (xwith + ywith) / wheel_radius) {}

    /* ---------------------------------- 速度 ---------------------------------- */

    /// @brief 速度正解：底盘速度 -> 四轮角速度
    /// @param v 底盘速度 (vx, vy, wz)
    /// @param w 输出轮角速度 [FL, FR, RL, RR] (rad/s)
    [[nodiscard]] Wheel WheelFromTwist(const Twist& v) const {
        const float w1 = u_ * (v.vx - v.vy - ab_ * v.wz);
        const float w2 = u_ * (v.vx + v.vy + ab_ * v.wz);
        const float w3 = u_ * (v.vx + v.vy - ab_ * v.wz);
        const float w4 = u_ * (v.vx - v.vy + ab_ * v.wz);

        return {w1, w2, w3, w4};
    }

    /// @brief 速度逆解：四轮角速度 -> 底盘速度（里程计）
    /// @param w 轮角速度 [FL, FR, RL, RR] (rad/s)
    /// @return 底盘速度 (vx, vy, wz)
    [[nodiscard]] Twist TwistFromWheel(const float (&w)[4]) const {
        const float s = w[0] + w[1] + w[2] + w[3];
        const float d = -w[0] + w[1] + w[2] - w[3];
        const float e = -w[0] + w[1] - w[2] + w[3];

        return {r_ * s, r_ * d, rab_ * e};
    }

    /* ----------------------------------- 力 ----------------------------------- */

    /// @brief 力正解：底盘期望力旋量 -> 四轮力矩（最小范数，不含内力）
    /// @param f 底盘力旋量 (fx, fy, mz)
    /// @param tau 输出轮力矩 [FL, FR, RL, RR] (N·m)
    [[nodiscard]] Wheel TorqueFromWrench(const Wrench& f) const {
        const float mz = f.mz / ab_;

        const float w1 = r_ * (f.fx - f.fy - mz);
        const float w2 = r_ * (f.fx + f.fy + mz);
        const float w3 = r_ * (f.fx + f.fy - mz);
        const float w4 = r_ * (f.fx - f.fy + mz);

        return {w1, w2, w3, w4};
    }

    /// @brief 力逆解：四轮力矩 -> 底盘力旋量
    /// @param tau 轮力矩 [FL, FR, RL, RR] (N·m)
    /// @return 底盘力旋量 (fx, fy, mz)
    [[nodiscard]] Wrench WrenchFromTorque(const float (&tau)[4]) const {
        const float s = tau[0] + tau[1] + tau[2] + tau[3];
        const float d = -tau[0] + tau[1] + tau[2] - tau[3];
        const float e = -tau[0] + tau[1] - tau[2] + tau[3];

        return {u_ * s, u_ * d, uab_ * e};
    }
};

} // namespace vlib

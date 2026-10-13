/**
 * @file pid.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-10-13
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
 * @note
 * 注意三种PID各自只维护各自需要的prev error，同时混用有风险
*/

#pragma once

#include <type_traits>

namespace vlib {

template<typename T = float>
class PID {
    static_assert(std::is_floating_point<T>::value, "PID requires floating point type");

    T kp, ki, kd;
    T maxout, maxiout;
    T dt;

    T e {}, pe {}, ppe {}; // p 指 prev
    T inte {}, diffe {};
    T out {}, pout {}, iout {}, dout {};

public:
    PID(T kp_, T ki_, T kd_, T maxout_, T maxiout_, T dt_ = T {1}):
        kp(kp_),
        ki(ki_),
        kd(kd_),
        maxout(maxout_ >= T {0} ? maxout_ : T {0}),
        maxiout(maxiout_ >= T {0} ? maxiout_ : T {0}),
        dt(dt_ > T {0} ? dt_ : T {1}) {}

    /// @brief 普通位置式 PID
    /// @param reference 参考值（目标值）
    /// @param measure 测量值（反馈值）
    /// @return control signal
    T Update(const T& reference, const T& measure) {
        pe = e;
        e  = reference - measure;

        inte += e * dt;
        diffe = (e - pe) / dt;

        pout    = kp * e;
        T iout_ = ki * inte;
        iout    = ClampABS(iout_, maxiout);
        dout    = kd * diffe;
        out     = ClampABS(pout + iout + dout, maxout);

        if (iout_ != iout_) {
            inte -= e * dt;
        }

        return out;
    }

    /// @brief 外部维护的 PID
    /// @param e_ 误差
    /// @param inte_ 积分
    /// @param diffe_ 微分
    T UpdateEZ(const T& e_, const T& inte_, const T& diffe_) {
        e     = e_;
        inte  = inte_;
        diffe = diffe_;

        pout = kp * e;
        iout = ClampABS(ki * inte, maxiout);
        dout = kd * diffe;
        out  = ClampABS(pout + iout + dout, maxout);

        return out;
    }

    /// @brief 差分 PID（增量 PID）
    /// @note 返回的是增量控制信号
    T UpdateDiff(const T& reference, const T& measure) {
        ppe = pe;
        pe  = e;
        e   = reference - measure;

        pout = kp * (e - pe);
        iout = ki * e * dt;
        dout = kd * (e - T {2} * pe + ppe) / dt;
        out  = ClampABS(pout + iout + dout, maxout);

        return out;
    }

    PID& SetDT(T dt_) {
        dt = dt_ <= T {0} ? dt : dt_;
        return *this;
    }

    PID& SetParam(T kp_, T ki_, T kd_) {
        kp = kp_;
        ki = ki_;
        kd = kd_;
        return *this;
    }

    PID& Reset() {
        e = pe = ppe = T {};
        inte = diffe = T {};
        out = pout = iout = dout = T {};
        return *this;
    }

    static constexpr T ClampABS(T x, T limit) {
        return x > limit ? limit : (x < -limit ? -limit : x);
    }
};

} // namespace vlib

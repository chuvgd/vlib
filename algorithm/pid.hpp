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

class PID {
    static_assert(std::is_floating_point<float>::value, "PID requires floating point type");

    float kp, ki, kd;
    float maxout, maxiout;
    float dt;

    float e {}, pe {}, ppe {}; // p 指 prev
    float inte {}, diffe {};
    float out {}, pout {}, iout {}, dout {};

public:
    PID(float kp_, float ki_, float kd_, float maxout_, float maxiout_, float dt_ = float {1}):
        kp(kp_),
        ki(ki_),
        kd(kd_),
        maxout(maxout_ >= float {0} ? maxout_ : float {0}),
        maxiout(maxiout_ >= float {0} ? maxiout_ : float {0}),
        dt(dt_ > float {0} ? dt_ : float {1}) {}

    /// @brief 普通位置式 PID
    /// @param reference 参考值（目标值）
    /// @param measure 测量值（反馈值）
    /// @return control signal
    float Update(const float& reference, const float& measure) {
        pe = e;
        e  = reference - measure;

        inte  += e * dt;
        diffe  = (e - pe) / dt;

        pout        = kp * e;
        float iout_ = ki * inte;
        iout        = ClampABS(iout_, maxiout);
        dout        = kd * diffe;
        out         = ClampABS(pout + iout + dout, maxout);

        if (iout_ != iout_) {
            inte -= e * dt;
        }

        return out;
    }

    /// @brief 外部维护的 PID
    /// @param e_ 误差
    /// @param inte_ 积分
    /// @param diffe_ 微分
    float UpdateEZ(const float& e_, const float& inte_, const float& diffe_) {
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
    float UpdateDiff(const float& reference, const float& measure) {
        ppe = pe;
        pe  = e;
        e   = reference - measure;

        pout = kp * (e - pe);
        iout = ki * e * dt;
        dout = kd * (e - float {2} * pe + ppe) / dt;
        out  = ClampABS(pout + iout + dout, maxout);

        return out;
    }

    PID& SetDT(float dt_) {
        dt = dt_ <= float {0} ? dt : dt_;
        return *this;
    }

    PID& SetParam(float kp_, float ki_, float kd_) {
        kp = kp_;
        ki = ki_;
        kd = kd_;
        return *this;
    }

    PID& Reset() {
        e = pe = ppe = float {};
        inte = diffe = float {};
        out = pout = iout = dout = float {};
        return *this;
    }

    static constexpr float ClampABS(float x, float limit) { return x > limit ? limit : (x < -limit ? -limit : x); }
};

} // namespace vlib

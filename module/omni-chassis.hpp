/**
 * @file omni-chassis.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief 全向运动底盘（mecanum wheel & omni wheel & steer wheel）
 * @version 0.1.0
 * @date 2026-07-25
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef OMNI_CHASSIS_HPP
#define OMNI_CHASSIS_HPP

namespace rb2 {
namespace module {

class Omni_Chassis {
public:
    enum class Type {
        M4,   // 麦
        O4,   // 4全
        O3,   // 3全
        S4,   // 4舵
        SSOO, // 对称半舵半全
        SOSO, // 对角半舵半全
    };

    Type type;

    Omni_Chassis(Type t): type(t) {}
};

} // namespace module

} // namespace rb2

#endif

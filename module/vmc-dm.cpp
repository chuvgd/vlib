/**
 * @file vmc-dm.cpp
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-19
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "vmc-dm.hpp"
#include "arm_math.h"
#include "math.h"

#define SINF(x) arm_sin_f32(x)
#define COSF(x) arm_cos_f32(x)

namespace rb2 {
namespace module {

// 给杆长赋值
// l1 = 0.15f; // 单位为m
// l2 = 0.25f; // 单位为m
// l3 = 0.25f; // 单位为m
// l4 = 0.15f; // 单位为m
// l5 = 0.15f; // AE长度 //单位为m
VMC_DM::VMC_DM(float l1, float l2, float l3, float l4, float l5):
    l1(l1),
    l2(l2),
    l3(l3),
    l4(l4),
    l5(l5) {
    first_flag = 0;

    is_offground = false;
}

// 计算theta和d_theta给lqr用，同时也计算腿长L0
void VMC_DM::VMC_5bar_FK(float phi1, float phi4, float pitch, float dpitch, float dt) {
    this->phi1 = phi1;
    this->phi4 = phi4;

    YD = l4 * SINF(phi4);      // D的y坐标
    YB = l1 * SINF(phi1);      // B的y坐标
    XD = l5 + l4 * COSF(phi4); // D的x坐标
    XB = l1 * COSF(phi1);      // B的x坐标

    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));

    A0 = 2 * l2 * (XD - XB);
    B0 = 2 * l2 * (YD - YB);
    C0 = l2 * l2 + lBD * lBD - l3 * l3;
    phi2 = 2 * atan2f((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    phi3 = atan2f(YB - YD + l2 * SINF(phi2), XB - XD + l2 * COSF(phi2));
    // C点直角坐标
    XC = l1 * COSF(phi1) + l2 * COSF(phi2);
    YC = l1 * SINF(phi1) + l2 * SINF(phi2);
    // C点极坐标
    L0 = sqrt((XC - l5 / 2.0f) * (XC - l5 / 2.0f) + YC * YC);

    phi0 = atan2f(YC, (XC - l5 / 2.0f)); // phi0用于计算lqr需要的theta
    alpha = PI / 2.0f - phi0;

    if (first_flag == 0) {
        last_phi0 = phi0;
        first_flag = 1;
    }
    d_phi0 = (phi0 - last_phi0) / dt; // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
    d_alpha = 0.0f - d_phi0;

    // 计算 theta
    last_theta = theta;
    theta = PI / 2.0f - pitch - phi0; // 得到状态变量1
    d_theta = (-dpitch - d_phi0);     // 得到状态变量2
    // d_theta = (theta - last_theta) / dt;
    dd_theta = (d_theta - last_d_theta) / dt;
    last_d_theta = d_theta;

    last_phi0 = phi0;

    d_L0 = (L0 - last_L0) / dt;      // 腿长L0的一阶导数
    dd_L0 = (d_L0 - last_d_L0) / dt; // 腿长L0的二阶导数

    last_d_L0 = d_L0;
    last_L0 = L0;
}

// 计算期望的关节输出力矩
void VMC_DM::VMC_5bar_IK(float tp, float f0) {
    Tp = tp;
    F0 = f0;

    // 计算雅可比
    j11 = (l1 * SINF(phi0 - phi3) * SINF(phi1 - phi2)) / SINF(phi3 - phi2);
    j12 = (l1 * COSF(phi0 - phi3) * SINF(phi1 - phi2)) / (L0 * SINF(phi3 - phi2));
    j21 = (l4 * SINF(phi0 - phi2) * SINF(phi3 - phi4)) / SINF(phi3 - phi2);
    j22 = (l4 * COSF(phi0 - phi2) * SINF(phi3 - phi4)) / (L0 * SINF(phi3 - phi2));

    // 得到RightFront的输出轴期望力矩，F0为五连杆机构末端沿腿的推力，Tp为沿中心轴的力矩
    torque_set[FRONT] = j11 * F0 + j12 * Tp;
    torque_set[BACK] = j21 * F0 + j22 * Tp;
}

#define OFFGROUND_FN_THRESHOLD 0.0f // 离地支持力阈值
#define OFFGROUND_A_RATIO -0.6f     // 加速度比例

bool VMC_DM::OffGround_Detection(float az) {
    fn =                 //
        F0 * COSF(theta) //
        + Tp * SINF(theta) / L0
        + OFFGROUND_A_RATIO
            * (az                                    //
               + (-dd_L0 * COSF(theta))              //
               + 2.0f * d_L0 * d_theta * SINF(theta) //
               + L0 * dd_theta * SINF(theta)         //
               + L0 * d_theta * d_theta * COSF(theta));

    is_offground = fn < OFFGROUND_FN_THRESHOLD;

    return is_offground;
}

} // namespace module
} // namespace rb2

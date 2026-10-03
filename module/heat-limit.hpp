/**
 * @file heat-limit.hpp
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-06-21
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef HEAT_LIMIT_HPP
#define HEAT_LIMIT_HPP

#include "stdint.h"
#include "math.h"

template<
    int BULLET_HEAT = 10,       // 单发热量
    int GEAR_RATIO = 36,        // 2006减速比
    int SMALL_GEAR_RATIO_ = 25, // 大小齿轮比2：5 * 10
    int SLOTS_ON_PLATE = 10,    // 拨盘弹丸数
    int COOLING_SAFE_GAP = 21,  // 热量留余
    int SHOOT_MOTOR_DARTLE_SPEED_SET = 500>
/// @brief 热量限制
/// @param cooling_speed 冷却速度
/// @param heat_limit 最大热量
/// @param heat_per_bullet 单发热量
/// @return 射速
float Heat_Limit(float cooling_speed, float heat_limit, float heat_per_bullet) {
    constexpr float SMALL_GEAR_RATIO = SMALL_GEAR_RATIO_ * 0.1f;

    float shoot_freq = 0;
    // if (Mode.Shoot_Mode == SINGLE || Mode.Fire_Mode == FIRE_REVERSAL)
    // 	return;

    // float a_cooling = REF.GameRobotStat.shooter_barrel_cooling_value;
    float current_heat = heat_limit - heat_per_bullet;

    if (1 /* is fire */) {
        // 参数合法性保护
        if (cooling_speed <= 0.0f || heat_per_bullet <= 0.0f) {
            shoot_freq = 0.0f;
        } else {
            float m_left = current_heat - COOLING_SAFE_GAP;
            float target_fire_rate = 0.0f;

            if (m_left > 0)
                target_fire_rate = (cooling_speed / BULLET_HEAT) + (m_left * 0.1f);
            else
                target_fire_rate = cooling_speed / BULLET_HEAT;

            float max_safe_rpm =
                (target_fire_rate * 60.0f / SLOTS_ON_PLATE) * GEAR_RATIO * SMALL_GEAR_RATIO;

            shoot_freq = std::fmax(
                0.0f,
                std::fmin(max_safe_rpm, static_cast<float>(SHOOT_MOTOR_DARTLE_SPEED_SET))
            );
        }
    } else {
        shoot_freq = 0;
    }

    return shoot_freq;
}

#endif

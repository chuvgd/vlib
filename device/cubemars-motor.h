/**
 * @file cubemars-motor.h
 * @author Serialist (ba3pt@qq.com)
 * @brief 
 * @version 0.1.0
 * @date 2026-05-13
 * 
 * @copyright Copyright (c) Serialist 2026
 * 
*/

#ifndef CUBEMARS_MOTOR_H
#define CUBEMARS_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "utils/bsp-adapter.h"
#include "utils/utils.h"

#define P_MAX 12.5f
#define V_MAX 50.0f
#define T_MAX 65.0f
#define Kp_MIN 0
#define Kp_MAX 500.0f
#define Kd_MIN 0
#define Kd_MAX 5.0f

#define V_MAX_2 45.0f
#define T_MAX_2 15.0f

// 伺服模式//
// void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
// void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
// void buffer_append_int16(uint8_t *buffer, int16_t number, int16_t *index);
// void comm_can_set_current(uint8_t controller_id, float current);
// void comm_can_set_rpm(uint8_t controller_id, float rpm);
// void comm_can_set_pos(uint8_t controller_id, float pos);
// void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode);

typedef struct {
    uint8_t id;
    float angle;   // 位置
    float speed;   // 速度
    float torque;  // 扭矩
    float temp;    // 温度
    float errCode; // 故障码
    uint8_t cnt;   // 在线监测倒计时
    bool isOnline; // 在线状态
} Motor_AK_RxData_t;

// 运控模式//
void AK_Motor_MIT_Enable(BSP_Port_t port, uint8_t id);
void AK_Motor_MIT_Disable(BSP_Port_t port, uint8_t id);
void AK_Motor_MIT_Setorigin(BSP_Port_t port, uint8_t id);
void AK_Motor_MIT_Transmit(
    BSP_Port_t port,
    uint8_t id,
    float p_des,
    float v_des,
    float kp,
    float kd,
    float t_ff
);
void AK_Motor_MIT_Decode(Motor_AK_RxData_t* data, uint8_t* buf, float pMax, float vMax, float tMax);

void AK_Motor_Monitor(Motor_AK_RxData_t* data);

inline bool AK_Motor_IsOnline(Motor_AK_RxData_t* data) {
    return data->isOnline;
}

#define AK10_9_MIT_Feedback_Decode(buf, data) AK_Motor_MIT_Decode(data, buf, P_MAX, V_MAX, T_MAX)

#ifdef __cplusplus
}
#endif

#endif

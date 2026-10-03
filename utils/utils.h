/**
 * @file utils.h
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2026-01-12
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================ include ================================================================ */

#include "math.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#include "robo-lib-config.h"

#ifdef ARM_MATH
    #include "arm_math.h"
#endif

// #include "cmsis_os.h" // 下面有这个要不要用（?）

/* ================================================================ macro ================================================================ */

#if !defined(user_malloc) && defined(_CMSIS_OS_H)
    #define user_malloc pvPortMalloc
#else
    #define user_malloc malloc
#endif

#ifndef PI
    #define PI 3.14159265354f
#endif

#ifndef EULER_NUMBER
    #define EULER_NUMBER 2.718281828f
#endif

#ifndef MIN
    #define MIN(a, b) ((a) <= (b) ? (a) : (b))
#endif

#ifndef MAX
    #define MAX(a, b) ((a) >= (b) ? (a) : (b))
#endif

#define DEG_CLAMPF(Ang) LoopClampf((Ang), -180.0f, 180.0f) // 角度格式化为-180~180
#define RAD_CLAMPF(Ang) LoopClampf((Ang), -PI, PI)         // 弧度格式化为-PI~PI

#ifndef DEG2RAD
    #define DEG2RAD(Ang) ((Ang) * 0.01745329252f)
#endif

#ifndef RAD2DEG
    #define RAD2DEG(Ang) ((Ang) * 57.295779513f)
#endif

#define Angle_to_rad 0.01745329f
#define Rad_to_angle 57.2957732f

#define RadiansToDegrees 57.295779513f
#define DegreesToRadians 0.01745329251f

#define Euler_Number 2.718281828459045f

#ifndef LF
    #define LF 0
    #define LB 1
    #define RF 2
    #define RB 3
#endif

// 轮子
#define WL 4
#define WR 5

#define FRONT 0
#define BACK 1

#define LEFT 0
#define RIGHT 1

#ifndef NONE
    #define NONE 0
#endif

#ifndef NULL
    #define NULL 0
#endif

// 做一层 adapter，方便移植
/// @todo 放到单独一个适配层文件中，比如 math-adapter，这样不同平台移植方便还能硬件优化
#define SINF(x) arm_sin_f32(x)
#define COSF(x) arm_cos_f32(x)
#define SQRTF(in, pout) arm_sqrt_f32(in, pout)
#define SIGNBIT(x) signbit(x)

#if defined(BOARD_DM_MC02)

    // 用作 buffer
    // h7 用，因为 buf 需要实时不能 cache，所以把这块内存放在 RAM1 中发一个 axi_sram 块中，这个块在 .sct 文件中会定义
    #define BUFFER_T __attribute__((section(".AXI_SRAM"))) uint8_t

    // 自己定义的符号名，以后改成这个吧
    #define MAT32 arm_matrix_instance_f32
    #define MAT32_INIT arm_mat_init_f32
    #define MAT32_ADD arm_mat_add_f32
    #define MAT32_SUB arm_mat_sub_f32
    #define MAT32_MULT arm_mat_mult_f32
    #define MAT32_TRANS arm_mat_trans_f32
    #define MAT32_INVERSE arm_mat_inverse_f32

#elif defined(BOARD_RM_C)

    #define BUFFER_T uint8_t

    #define MAT32 arm_matrix_instance_f32
    #define MAT32_INIT arm_mat_init_f32
    #define MAT32_ADD arm_mat_add_f32
    #define MAT32_SUB arm_mat_sub_f32
    #define MAT32_MULT arm_mat_mult_f32
    #define MAT32_TRANS arm_mat_trans_f32
    #define MAT32_INVERSE arm_mat_inverse_f32

#endif

/* ================================================================ typedef ================================================================ */

/* ================================================================ variable ================================================================ */

/* ================================================================ prototype ================================================================ */

float Signf(float value);                                                    // 符号函数
void Clampfp(float* in, float min, float max);                               // 指针限幅
float Clampf(float value, float min, float max);                             // 限幅
float ClampAbsf(float value, float max);                                     // 绝对值限幅
float LoopClampf(float Input, float minValue, float maxValue);               // 循环限幅
float Remapf(float a, float inmin, float inmax, float outmin, float outmax); // 值映射
float Rampf(float prev_x, float x, float k_min, float k_max, float dt);      // 斜坡函数
float Deadzonef(float value, float point, float deadzone);                   // 死区

float Bit2Float(int x_int, float x_min, float x_max, int Bits);
int Float2Bit(float x, float x_min, float x_max, int bits);

float Modf(float value, float range);                     // 取模
float SSqrt(float x);                                     // 开方
long long FPow(long long a, long long b);                 // 快速幂
long long FPowMod(long long a, long long b, long long p); // 快速幂取模
float FiSqrt(float x);                                    // 快速平方根倒数
float FSqrtf(float x);                                    // 快速平方根
long long FGcd(long long a, long long b);                 // 最大公约数 greatest common divisor

/* ================================================================ function ================================================================ */

#ifdef __cplusplus
}
#endif

#endif

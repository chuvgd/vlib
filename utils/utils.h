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

#ifndef DEG2RAD
    #define DEG2RAD(Ang) ((Ang) * 0.01745329252f)
#endif

#ifndef RAD2DEG
    #define RAD2DEG(Ang) ((Ang) * 57.295779513f)
#endif

// index 简写

#define ILF 0 // 左前
#define ILB 1 // 左后
#define IRF 2 // 右前
#define IRB 3 // 右后

#define IFRONT 0
#define IBACK 1

#define ILEFT 0
#define IRIGHT 1

#ifdef __cplusplus
}
#endif

#endif

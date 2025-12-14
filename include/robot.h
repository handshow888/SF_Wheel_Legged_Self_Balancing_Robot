#pragma once

#include "CAN/CAN_comm.h"

// 连杆长度参数
#define L1 150 // 后大腿长
#define L2 250 // 后小腿长
#define L3 250 // 前小腿长
#define L4 150 // 前大腿长
#define L5 120 // 关节电机水平距离

typedef struct
{
    float x, y;
} Point;

typedef struct
{
    float alpha; // 后关节电机角度 rad
    float beta;  // 前关节电机角度 rad
    float theta1;
    float theta2;
    Point A; // 后大小腿关节连接处
    Point B; // 足端点
    Point C; // 前大小腿关节连接处

} LegKinematicsParams;


extern LegKinematicsParams leftLegKinematics, rightLegKinematics;   // 左右腿运动学参数
extern Point leftEndTarget, rightEndTarget; // 左右腿足端目标点
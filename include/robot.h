#pragma once
#include "Arduino.h"
#include "CAN/CAN_comm.h"
#include "CAN/can.h"
#include "ppm.h"
#include "pid.h"

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
    // float theta1;
    // float theta2;
    // Point A; // 后大小腿关节连接处
    // Point B; // 足端点
    // Point C; // 前大小腿关节连接处

} LegKinematicsParams;

extern LegKinematicsParams leftLegKinematics, rightLegKinematics; // 左右腿运动学参数
extern Point leftLegEndTarget, rightLegEndTarget;                 // 左右腿足端目标点

extern float motor1_vel, motor2_vel; // 两个轮毂电机的轮速 rad/s

extern float remoteLinearVel;          // 从遥控器接收的前进后退速度
extern float remoteSteering;           // 从遥控器接收的左右转向速度
extern float remoteBalanceOffset;      // 从遥控器接收的平衡pitch偏移量
extern float remoteLegHeight;          // 从遥控器接收的腿高 单位：mm
extern float remoteShakeShoulderValue; // 从遥控器接收的抖肩值

extern const int legHeightMin; // 腿高最低值
extern const int legHeightMax; // 腿高最高值

extern float rightWheelTorTarget; // 右轮毂电机目标扭矩
extern float leftWheelTorTarget;  // 左轮毂电机目标扭矩

extern const float wheelRadius; // 轮子半径

void legEndCalculate();
void jumpControl();
void wheelControlLQR();

void mapPPMToRobotControl();
float mapJoyStickValueCenter(int inputValue, float scale);
float mapJoyStickValueKnob(int inputValue, float scale);
int mapJoyStickValueHeight(int inputValue);

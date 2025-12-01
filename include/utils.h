#pragma once
#include <Arduino.h>

#define clamp(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

float lowPassFilter(float currentValue, float previousValue, float alpha);
float invSqrt(float x);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);

constexpr float deg2rad = PI / 180.0f;  // 乘以此变量可从角度变换到弧度
constexpr float rad2deg = 180.0f / PI;// 乘以此变量可从弧度变换到角度
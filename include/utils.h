#pragma once
#include <Arduino.h>
#include <BasicLinearAlgebra.h>

#define clamp(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

float lowPassFilter(float currentValue, float previousValue, float alpha);
float invSqrt(float x);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);

constexpr float deg2rad = PI / 180.0f;  // 乘以此变量可从角度变换到弧度
constexpr float rad2deg = 180.0f / PI;// 乘以此变量可从弧度变换到角度

// 打印任意 M×N 矩阵
template <int M, int N>
void PrintMatrix(const BLA::Matrix<M, N> &mat, const char *name = nullptr)
{
    if (name)
    {
        Serial.print(name);
        Serial.println(":");
    }
    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < N; j++)
        {
            Serial.print(mat(i, j), 6); // 保留6位小数
            if (j < N - 1)
                Serial.print(",\t");
        }
        Serial.println();
    }
    Serial.println();
}
#include "pid.h"

PidParams pidVel = {0, 0};     // 速度环pid
PidParams pidBalance = {0, 0}; // 直立环pid
PidParams pidLegX = {0, 0};    // 腿的足端X坐标pid

/**
 * @brief pid计算轮毂电机扭矩
 */
void wheelControlPid()
{
    float rightWheelVel = -motor1_vel; // 左轮轮速 rad/s
    float leftWheelVel = -motor2_vel;  // 右轮轮速 rad/s
    float currentVel = (leftWheelVel + rightWheelVel) * wheelRadius * 0.5f;

    float pitchTarget = pidVel.kp * (remoteLinearVel - currentVel);
    float wheelTorTarget = pidBalance.kp * (pitchTarget - INS.Pitch * deg2rad + remoteBalanceOffset) + pidBalance.kd * mpu6050.getGyroY() * deg2rad;
    wheelTorTarget = clamp(wheelTorTarget, -5, 5);

    rightWheelTorTarget = wheelTorTarget + remoteSteering;
    leftWheelTorTarget = wheelTorTarget - remoteSteering;
    rightWheelTorTarget = clamp(rightWheelTorTarget, -5, 5);
    leftWheelTorTarget = clamp(leftWheelTorTarget, -5, 5);
}

/**
 * @brief pid插值拟合函数
 */
void interpolatePID()
{
    // 选取的三个特定腿高度
    const float y0 = legHeightMin;
    const float y1 = (legHeightMin + legHeightMax) * 0.5;
    const float y2 = legHeightMax;

    BipedalPids pid0 = {1, 1, 1, 1, 0};
    BipedalPids pid1 = {1, 1, 1, 1, 0};
    BipedalPids pid2 = {1, 1, 1, 1, 0};

    if (remoteLegHeight <= y1)
    {
        float ratio = (remoteLegHeight - y0) / (y1 - y0);
        pidVel.kp = pid0.velKp + ratio * (pid1.velKp - pid0.velKp);
        pidBalance.kp = pid0.balanceKp + ratio * (pid1.balanceKp - pid0.balanceKp);
        pidBalance.kd = pid0.balanceKd + ratio * (pid1.balanceKd - pid0.balanceKd);
        pidLegX.kp = pid0.legXKp + ratio * (pid1.legXKp - pid0.legXKp);
        pidLegX.kd = pid0.legXKd + ratio * (pid1.legXKd - pid0.legXKd);
    }
    else
    {
        float ratio = (remoteLegHeight - y1) / (y2 - y1);
        pidVel.kp = pid1.velKp + ratio * (pid2.velKp - pid1.velKp);
        pidBalance.kp = pid1.balanceKp + ratio * (pid2.balanceKp - pid1.balanceKp);
        pidBalance.kd = pid1.balanceKd + ratio * (pid2.balanceKd - pid1.balanceKd);
        pidLegX.kp = pid1.legXKp + ratio * (pid2.legXKp - pid1.legXKp);
        pidLegX.kd = pid1.legXKd + ratio * (pid2.legXKd - pid1.legXKd);
    }
}
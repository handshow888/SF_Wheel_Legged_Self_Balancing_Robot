#pragma once
#include "Arduino.h"
#include "robot.h"
#include "tasks/ins_task.h"
#include "MPU6050.h"

typedef struct
{
    float kp;
    float kd;
} PidParams;

typedef struct
{
    float velKp;
    float balanceKp;
    float balanceKd;
    float legXKp;
    float legXKd;
} BipedalPids;

extern PidParams pidVel;     // 速度环pid
extern PidParams pidBalance; // 直立环pid
extern PidParams pidLegX;    // 腿的足端X坐标pid

void wheelControlPid();
void interpolatePID();
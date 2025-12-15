#pragma once
#include "Arduino.h"
#include "robot.h"
#include "utils.h"

Point forwardKinematics(float alpha, float beta);
void inverseKinematics(LegKinematicsParams &leftLeg, LegKinematicsParams &rightLeg, Point &leftTarget, Point &rightTarget);

static inline float wrapTo2Pi(float radian);
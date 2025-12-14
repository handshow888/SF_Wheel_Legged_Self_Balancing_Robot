#pragma once
#include "Arduino.h"
#include "robot.h"

void forwardKinematics();                                                                               // 正解
void inverseKinematics(LegKinematicsParams *leftLeg, LegKinematicsParams *rightLeg, Point *leftTarget, Point *rightTarget); // 逆解
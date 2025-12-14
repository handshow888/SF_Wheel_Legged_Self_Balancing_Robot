#include "robot.h"

MIT devicesState[8];

LegKinematicsParams leftLegKinematics, rightLegKinematics;   // 左右腿运动学参数
Point leftEndTarget, rightEndTarget; // 左右腿足端目标点
#include "robot.h"

MIT devicesState[8];

LegKinematicsParams leftLegKinematics, rightLegKinematics; // 左右腿运动学参数
Point leftLegEndTarget, rightLegEndTarget;                 // 左右腿足端目标点

float motor1_vel = 0, motor2_vel = 0;
float remoteLinearVel;          // 从遥控器接收的前进后退速度
float remoteSteering;           // 从遥控器接收的左右转向速度
float remoteBalanceOffset;      // 从遥控器接收的平衡pitch偏移量
float remoteLegHeight;          // 从遥控器接收的腿高 单位：mm
float remoteShakeShoulderValue; // 从遥控器接收的抖肩值

const int legHeightMin = 140; // 腿高最低值
const int legHeightMax = 300; // 腿高最高值

float rightWheelTorTarget; // 右轮毂电机目标扭矩
float leftWheelTorTarget;  // 左轮毂电机目标扭矩

/**
 * @brief 计算足端目标坐标（逆解初值）
 */
void legEndCalculate()
{
    const int jumpLegHeight = 150;
    jumpControl();
    switch (knobMode)
    {
    case PPM_KNOB_MODE::Normal:
        leftLegEndTarget.y = remoteLegHeight;
        rightLegEndTarget.y = remoteLegHeight;
        break;
    case PPM_KNOB_MODE::ShakeShoulder:
        leftLegEndTarget.y = remoteLegHeight - remoteShakeShoulderValue;
        rightLegEndTarget.y = remoteLegHeight + remoteShakeShoulderValue;
        leftLegEndTarget.y = clamp(leftLegEndTarget.y, legHeightMin, legHeightMax);
        rightLegEndTarget.y = clamp(rightLegEndTarget.y, legHeightMin, legHeightMax);
        break;
    case PPM_KNOB_MODE::Jump:
        if (jumpProcess == JUMPING_PROCESS::JUMPING)
        {
            leftLegEndTarget.y = remoteLegHeight + jumpLegHeight;
            rightLegEndTarget.y = leftLegEndTarget.y;
        }
        else
        {
            leftLegEndTarget.y = remoteLegHeight;
            rightLegEndTarget.y = remoteLegHeight;
        }
        break;
    default:
        break;
    }
    float currentVel = ((-motor1_vel) + (-motor2_vel)) * 0.5f;
    leftLegEndTarget.x = L4 / 2 + pidLegX.kp * -(remoteLinearVel - currentVel);
    leftLegEndTarget.x = clamp(leftLegEndTarget.x, -100, L4 + 100);
    rightLegEndTarget.x = leftLegEndTarget.x;
}

/**
 * @brief 控制跳跃伸缩腿
 */
void jumpControl()
{
    static unsigned long startMillis = 0; // 用于记录起始时间
    switch (knobMode)
    {
    case PPM_KNOB_MODE::Jump:
        switch (jumpProcess)
        {
        case JUMPING_PROCESS::READY:
            Am_kp = 2.5;
            startMillis = millis();
            jumpProcess = JUMPING_PROCESS::JUMPING;
            break;
        case JUMPING_PROCESS::JUMPING:
            if (millis() - startMillis >= 100)
            {
                Am_kp = 1.3;
                jumpProcess = JUMPING_PROCESS::LAND;
            }
            break;
        case JUMPING_PROCESS::LAND:
            if (millis() - startMillis >= 2000)
            {
                Am_kp = 1.0;
            }
            break;
        default:
            break;
        }
        break;
    default:
        Am_kp = 1.0;
        jumpProcess = JUMPING_PROCESS::LAND;
        break;
    }
}

/////////////////////////////////////////////////////////
void mapPPMToRobotControl()
{
    remoteLinearVel = mapJoyStickValueCenter(PPM_RIGHT_STICK_UD, 0.01f);
    remoteSteering = mapJoyStickValueCenter(PPM_RIGHT_STICK_LR, 0.015f);
    remoteBalanceOffset = mapJoyStickValueKnob(PPM_RIGHT_KNOB, 0.01f);
    remoteLegHeight = mapJoyStickValueHeight(PPM_LEFT_STICK_UD);
    remoteShakeShoulderValue = mapJoyStickValueCenter(PPM_LEFT_STICK_LR, 1.0f / 15.0f);
}

/**
 * @brief 把摇杆数值映射成归中为0，区间±500的数值，再乘以缩放比例
 * @param inputValue 原始数值
 * @param scale 缩放比例
 */
float mapJoyStickValueCenter(int inputValue, float scale)
{
    inputValue = clamp(inputValue, 1000, 2000);
    if (inputValue > 1400 && inputValue < 1600)
        inputValue = 1500;
    float mappedValue = (inputValue - 1500) * scale;
    return mappedValue;
}

/**
 * @brief 把旋钮数值映射成归中为0，区间±500的数值，再乘以缩放比例
 * @param inputValue 原始数值
 * @param scale 缩放比例
 */
float mapJoyStickValueKnob(int inputValue, float scale)
{
    inputValue = clamp(inputValue, 1000, 2000);
    float mappedValue = (inputValue - 1500) * scale;
    if (mappedValue > -0.7 && mappedValue < 0.7)
    {
        mappedValue = 0;
    }
    return mappedValue;
}

/**
 * @brief 把左摇杆数值映射成腿高
 * @note 区间为 [legHeightMin, legHeightMax]
 * @retval remoteLegHeight 单位mm
 */
int mapJoyStickValueHeight(int inputValue)
{
    // 实际摇杆数值达不到1000和2000
    const int valueMin = 1100;
    const int valueMax = 1900;
    inputValue = clamp(inputValue, valueMin, valueMax);
    float ratio = (inputValue - valueMin) / (valueMax - valueMin);
    return (int)(ratio * (legHeightMax - legHeightMin) + legHeightMin);
}
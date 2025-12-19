#include "kinematics.h"
/**
 * @brief 运动学正解(关节电机角度解出足端坐标)
 */
Point forwardKinematics(float alpha, float beta)
{
    Point result;

    float x_a = L1 * cos(alpha);
    float y_a = L1 * sin(alpha);
    float x_c = L5 + L4 * cos(beta);
    float y_c = L4 * sin(beta);

    float a = 2 * (x_a - x_c) * L2;
    float b = 2 * (y_a - y_c) * L2;
    float L_AC = sqrt((x_a - x_c) * (x_a - x_c) + (y_a - y_c) * (y_a - y_c));
    float c = L3 * L3 - L2 * L2 - L_AC * L_AC;
    // theta1 ∈ [0, pi/2]
    float theta1 = 2 * atan((b + sqrt(a * a + b * b - c * c)) / (a + c));
    theta1 = wrapTo2Pi(theta1);

    if (!(theta1 >= 0 && theta1 <= PI / 2))
    {
        theta1 = 2 * atan((b - sqrt(a * a + b * b - c * c)) / (a + c));
        theta1 = wrapTo2Pi(theta1);
    }

    result.x = L1 * cos(alpha) + L2 * cos(theta1);
    result.y = L1 * sin(alpha) + L2 * sin(theta1);

    return result;
}

/**
 * @brief 运动学逆解(足端坐标解出关节电机角度)
 */
void inverseKinematics(LegKinematicsParams &leftLeg, LegKinematicsParams &rightLeg, Point &leftTarget, Point &rightTarget)
{
    // Serial.printf("target x:%.1f,y:%.1f\t", leftTarget.x, leftTarget.y);
    // alpha后电机角度  beta前电机角度
    float alpha, beta;

    /***** 右腿逆解运算 *****/
    // 解算alpha用
    float a = 2 * rightTarget.x * L1;
    float b = 2 * rightTarget.y * L1;
    float c = rightTarget.x * rightTarget.x + rightTarget.y * rightTarget.y + L1 * L1 - L2 * L2;
    // 解算beta用
    float d = 2 * L4 * (rightTarget.x - L5);
    float e = 2 * L4 * rightTarget.y;
    float f = ((rightTarget.x - L5) * (rightTarget.x - L5) + L4 * L4 + rightTarget.y * rightTarget.y - L3 * L3);

    // 一元二次方程第一个解
    alpha = 2 * atan((b + sqrt(a * a + b * b - c * c)) / (a + c));
    beta = 2 * atan((e + sqrt(d * d + e * e - f * f)) / (d + f));
    alpha = wrapTo2Pi(alpha);
    // beta = wrapTo2Pi(beta);

    // 如果解不在合理范围内，计算第二个解
    if (!(alpha >= PI / 4 && alpha <= (PI + PI / 6)))
    {
        // Serial.printf("alpha1: %.2f invalid\t", alpha / PI * 180.0f);
        alpha = 2 * atan((b - sqrt(a * a + b * b - c * c)) / (a + c));
        alpha = wrapTo2Pi(alpha);
    }
    if (!(beta >= -PI / 6 && beta <= PI / 2))
    {
        // Serial.printf("beta1: %.2f invalid\t", beta / PI * 180.0f);
        beta = 2 * atan((e - sqrt(d * d + e * e - f * f)) / (d + f));
        // beta = wrapTo2Pi(beta);
    }
    // Serial.printf("alpha:%.2f,beta%.2f\n", alpha / PI * 180.0f, beta / PI * 180.0f);
    rightLeg.alpha = alpha;
    rightLeg.beta = beta;

    /***** 左腿逆解运算 *****/
    if (leftTarget.x == rightTarget.x && leftTarget.y == rightTarget.y)
    {
        leftLeg.alpha = rightLeg.alpha;
        leftLeg.beta = rightLeg.beta;
    }
    else // 如果左右腿目标点不一样才需要进行左腿解算
    {
        // 解算alpha用
        a = 2 * leftTarget.x * L1;
        b = 2 * leftTarget.y * L1;
        c = leftTarget.x * leftTarget.x + leftTarget.y * leftTarget.y + L1 * L1 - L2 * L2;
        // 解算beta用
        d = 2 * L4 * (leftTarget.x - L5);
        e = 2 * L4 * leftTarget.y;
        f = ((leftTarget.x - L5) * (leftTarget.x - L5) + L4 * L4 + leftTarget.y * leftTarget.y - L3 * L3);

        // 一元二次方程第一个解
        alpha = 2 * atan((b + sqrt(a * a + b * b - c * c)) / (a + c));
        beta = 2 * atan((e + sqrt(d * d + e * e - f * f)) / (d + f));

        alpha = wrapTo2Pi(alpha);
        beta = wrapTo2Pi(beta);

        // 如果解不在合理范围内，计算第二个解
        if (!(alpha >= PI / 4 && alpha <= (PI + PI / 6)))
        {
            alpha = 2 * atan((b - sqrt(a * a + b * b - c * c)) / (a + c));
            alpha = wrapTo2Pi(alpha);
        }
        if (!(beta >= -PI / 6 && beta <= PI / 2))
        {
            beta = 2 * atan((e - sqrt(d * d + e * e - f * f)) / (d + f));
            beta = wrapTo2Pi(beta);
        }

        leftLeg.alpha = alpha;
        leftLeg.beta = beta;
    }
}

/**
 * @brief 把角度限制在[0,2PI]
 */
static inline float wrapTo2Pi(float radian)
{
    return (radian >= 0.0f) ? radian : (radian + 2 * PI);
}
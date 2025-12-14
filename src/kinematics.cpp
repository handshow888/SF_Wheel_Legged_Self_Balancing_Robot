#include "kinematics.h"

void inverseKinematics(LegKinematicsParams *leftLeg, LegKinematicsParams *rightLeg, Point *leftTarget, Point *rightTarget)
{
    // alpha后电机角度  beta前电机角度
    float alpha1, alpha2, beta1, beta2;

    /***** 右腿逆解运算 *****/
    // 解算alpha用
    float a = 2 * rightTarget->x * L1;
    float b = 2 * rightTarget->y * L1;
    float c = rightTarget->x * rightTarget->x + rightTarget->y * rightTarget->y + L1 * L1 - L2 * L2;
    // 解算beta用
    float d = 2 * L4 * (rightTarget->x - L5);
    float e = 2 * L4 * rightTarget->y;
    float f = ((rightTarget->x - L5) * (rightTarget->x - L5) + L4 * L4 + rightTarget->y * rightTarget->y - L3 * L3);

    // 一元二次方程的两个解
    alpha1 = 2 * atan((b + sqrt((a * a) + (b * b) - (c * c))) / (a + c));
    alpha2 = 2 * atan((b - sqrt((a * a) + (b * b) - (c * c))) / (a + c));
    beta1 = 2 * atan((e + sqrt((d * d) + e * e - (f * f))) / (d + f));
    beta2 = 2 * atan((e - sqrt((d * d) + e * e - (f * f))) / (d + f));

    alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

    if (alpha1 >= PI / 4 && alpha1 <= (PI + PI / 6))
        rightLeg->alpha = alpha1;
    else
        rightLeg->alpha = alpha2;
    if (beta1 >= -PI / 6 && beta1 <= PI / 2)
        rightLeg->beta = beta1;
    else
        rightLeg->beta = beta2;

    /***** 左腿逆解运算 *****/
    if (leftTarget->x == rightTarget->x && leftTarget->y == rightTarget->y)
    {
        leftLeg->alpha = rightLeg->alpha;
        leftLeg->beta = leftLeg->beta;
    }
    else // 如果左右腿目标点不一样才需要进行左腿解算
    {
        // 解算alpha用
        a = 2 * leftTarget->x * L1;
        b = 2 * leftTarget->y * L1;
        c = leftTarget->x * leftTarget->x + leftTarget->y * leftTarget->y + L1 * L1 - L2 * L2;
        // 解算beta用
        d = 2 * L4 * (leftTarget->x - L5);
        e = 2 * L4 * leftTarget->y;
        f = ((leftTarget->x - L5) * (leftTarget->x - L5) + L4 * L4 + leftTarget->y * leftTarget->y - L3 * L3);

        // 一元二次方程的两个解
        alpha1 = 2 * atan((b + sqrt((a * a) + (b * b) - (c * c))) / (a + c));
        alpha2 = 2 * atan((b - sqrt((a * a) + (b * b) - (c * c))) / (a + c));
        beta1 = 2 * atan((e + sqrt((d * d) + e * e - (f * f))) / (d + f));
        beta2 = 2 * atan((e - sqrt((d * d) + e * e - (f * f))) / (d + f));

        alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
        alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

        if (alpha1 >= PI / 4 && alpha1 <= (PI + PI / 6))
            leftLeg->alpha = alpha1;
        else
            leftLeg->alpha = alpha2;
        if (beta1 >= -PI / 6 && beta1 <= PI / 2)
            leftLeg->beta = beta1;
        else
            leftLeg->beta = beta2;
    }
}
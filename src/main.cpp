#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "tasks/ins_task.h"
// #include "bipedal_data.h"
#include "CAN/CAN_comm.h"
#include "CAN/can.h"
// #include "CAN/config.h"
// #include <WiFi.h>
// #include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ppm.h"
#include "Motor.h"
#include "robot.h"
#include "kinematics.h"
#include "pid.h"
#include "utils.h"

MPU6050 mpu6050 = MPU6050(Wire); // 实例化MPU6050

void IMUTask(void *pvParameters);
void Open_thread_function(); // 启动线程

void setup()
{
  Wire.begin(1, 2, 400000UL); // 初始化IIC
  Serial.begin(115200);       // 初始化调试串口
  mpu6050.begin();            // 初始化MPU陀螺仪
  mpu6050.setGyroOffsets(3.73, -1.59, -0.16);
  ppm_init(); // 遥控器读取中断初始化
  CANInit();
  delay(1000);
  enableJointMotors();    // 使能关节电机
  motorInit();            // 轮毂电机初始化
  Open_thread_function(); // 启动线程

  /* USER CALIBRATE IMU START */
  // 静止平放时解注释获取imu校准矩阵，正常运行时注释
  // delay(2);
  // mpu6050.calcGyroOffsets(true);
  // mpu6050.calibrateAccelerometer();
  /* USER CALIBRATE IMU END */
}

void loop()
{
  storeFilteredPPMData(); // 获取遥控器数据
  remoteSwitch();         // 获取遥控器模式
  mapPPMToRobotControl(); // 映射遥控器各通道数值为控制指令
  // interpolatePID();       // 根据腿高插值拟合pid参数

  legEndCalculate();                                                                             // 计算足端目标位置
  inverseKinematics(leftLegKinematics, rightLegKinematics, leftLegEndTarget, rightLegEndTarget); // 运动学逆解
  mapJointMotorAngle();                                                                          // 将运动学逆解的结果映射到关节电机角度pos
  CAN_Control();                                                                                 // 发送关节电机控制指令

  // wheelControlPID();                                                                           // pid计算轮毂电机扭矩
  wheelControlLQR(); // lqr计算轮毂电机扭矩
  sendMotorTargets(enableHubMotor * rightWheelTorTarget, enableHubMotor * leftWheelTorTarget); // 发送控制轮毂电机的目标值 右, 左
  // sendMotorTargets(enableHubMotor * remoteLinearVel, enableHubMotor * remoteLinearVel); // 发送控制轮毂电机的目标值 右, 左
  // Serial.printf("leftVel:%.2f\trightVel:%.2f\n", motor2_vel, motor1_vel);
}

// 启动线程
void Open_thread_function()
{
  // 陀螺仪读取任务进程
  xTaskCreatePinnedToCore(
      IMUTask,   // 任务函数
      "IMUTask", // 任务名称
      4096,      // 堆栈大小 4096 × 4 = 16384B
      NULL,      // 传递的参数
      1,         // 任务优先级
      NULL,      // 任务句柄
      1          // 运行在核心 0
  );
}

// 陀螺仪数据读取
void IMUTask(void *pvParameters)
{
  INS_Init();
  while (true)
  {
    INS_Task();
    // Serial.printf("Roll:%.3f\tPitch:%3f\tYaw:%.3f\n", INS.Roll, INS.Pitch, INS.Yaw);
  }
}

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "tasks/ins_task.h"
// #include "bipedal_data.h"
// #include "CAN/CAN_comm.h"
// #include "CAN/config.h"
// #include <WiFi.h>
// #include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "ppm.h"
// #include "Motor.h"
// #include "robot.h"
// #include "CAN/can.h"
// #include "pid.h"
#include "utils.h"

MPU6050 mpu6050 = MPU6050(Wire); // 实例化MPU6050

void IMUTask(void *pvParameters);
void Open_thread_function(); // 启动线程

void setup()
{
  Wire.begin(1, 2, 400000UL); // 初始化IIC
  Serial.begin(115200);       // 初始化调试串口
  mpu6050.begin();            // 初始化MPU陀螺仪
  Open_thread_function();     // 启动线程

  /* USER CALIBRATE IMU START */
  // 静止平放时解注释获取imu校准矩阵，正常运行时注释
  // mpu6050.calcGyroOffsets(true);
  // mpu6050.calibrateAccelerometer();
  /* USER CALIBRATE IMU END */
}

void loop()
{
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
  // gyroY = mpu6050.getGyroY(); // 第一次读出来作为初值
  // roll = mpu6050.getAngleX();
  // while (true)
  // {
  //   mpu6050.update();
  //   pitch = mpu6050.getAngleY();
  //   yaw = mpu6050.getAngleZ();
  //   gyroX = mpu6050.getGyroX();
  //   gyroZ = mpu6050.getGyroZ();
  //   // 这里是设置一定死区 避免数据的波动
  //   if (gyroZ > -8 && gyroZ < 8)
  //     gyroZ = 0;
  //   roll = lowPassFilter(mpu6050.getAngleX(), roll, 0.05);
  //   gyroY = lowPassFilter(mpu6050.getGyroY(), gyroY, 0.005);
  // }
  INS_Init();
  while (true)
  {
    INS_Task();
    Serial.printf("Roll:%.3f\tPitch:%3f\tYaw:%.3f\n", INS.Roll, INS.Pitch, INS.Yaw);
  }
  
}

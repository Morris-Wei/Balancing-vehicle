/******************************************************************************
MinimalistExample.ino

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

Description:
Most basic example of use.

Example using the LSM6DS3 with basic settings.  This sketch collects Gyro and
Accelerometer data every second, then presents it on the serial monitor.

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

Hardware connections:
Connect I2C SDA line to A4
Connect I2C SCL line to A5
Connect GND and 3.3v power to the IMU

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU; //Default constructor is I2C, addr 0x6B

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  //Call .begin() to configure the IMU
  myIMU.begin();
  
}

/*
void loop()
{
  //Get all parameters
  Serial.print("Accelerometer:");
  Serial.print(" X = ");
  Serial.print(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y = ");
  Serial.print(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z = ");
  Serial.print(myIMU.readFloatAccelZ(), 4);

  Serial.print("  Gyroscope:");
  Serial.print(" X = ");
  Serial.print(myIMU.readFloatGyroX(), 4);
  Serial.print(" Y = ");
  Serial.print(myIMU.readFloatGyroY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatGyroZ(), 4);
  /*
  Serial.print("\nThermometer:\n");
  Serial.print(" Degrees C = ");
  Serial.println(myIMU.readTempC(), 4);
  Serial.print(" Degrees F = ");
  Serial.println(myIMU.readTempF(), 4);
  
  delay(100);
}
*/

 

float ax, ay, az, gx, gy, gz;
 
//********************angle data*********************//
float Gyro_y; //Y轴陀螺仪数据暂存
float Gyro_x;
float Gyro_z;
float angleAx;
float angle6;
float K1 = 0.05; // 对加速度计取值的权重
float Angle; //一阶互补滤波计算出的小车最终倾斜角度
float accelz = 0;
 
//********************angle data*********************//
 
//***************Kalman_Filter*********************//
float P[2][2] = {{ 1, 0 },
  { 0, 1 }
};
float Pdot[4] = { 0, 0, 0, 0};
float Q_angle = 0.001, Q_gyro = 0.005; //角度数据置信度,角速度数据置信度
float R_angle = 0.5 , C_0 = 1;
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
float timeChange = 5; //滤波法采样时间间隔毫秒
float dt = timeChange * 0.001; //注意：dt的取值为滤波器采样时间
//***************Kalman_Filter*********************//
 
void Angletest()
{
  //平衡参数
  Angle = atan2(ay , az) * 57.3;           //角度计算公式
  Gyro_x = (gx - 128.1) / 131;              //角度转换
  Kalman_Filter(Angle, Gyro_x);            //卡曼滤波
  //旋转角度Z轴参数
  if (gz > 32768) gz -= 65536;              //强制转换2g  1g
  Gyro_z = -gz / 131;                      //Z轴参数转换
  accelz = az / 16.4;
 
  angleAx = atan2(ax, az) * 180 / PI; //计算与x轴夹角
  Gyro_y = -gy / 131.00; //计算角速度
  //一阶互补滤波
  angle6 = K1 * angleAx + (1 - K1) * (angle6 + Gyro_y * dt);
}
 
////////////////////////kalman/////////////////////////
float angle, angle_dot;                                //平衡角度值
void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err; //角度
  q_bias += K_1 * angle_err;
  angle_dot = gyro_m - q_bias; //角速度
}
 

 
void loop() {
 ax=myIMU.readFloatAccelX();     //IIC获取MPU6050六轴数据 ax ay az gx gy gz  
 ay=myIMU.readFloatAccelY();
 az=myIMU.readFloatAccelZ(); 
 gx=myIMU.readFloatGyroX();
 gy=myIMU.readFloatGyroY();
 gz=myIMU.readFloatGyroZ();
 Angletest();                                      //获取angle 角度和卡曼滤波
 
  Serial.print(ax);Serial.print(",");
  Serial.print(ay);Serial.print(",");
  Serial.print(az);Serial.print("---");
  Serial.print(angle);Serial.print(",");
  Serial.print(angle_dot);Serial.print(",");
  Serial.println(angle6);
  
  delay(5);
}


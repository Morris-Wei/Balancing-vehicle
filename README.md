# Balancing-vehicle
A project of 13th Pld contest in Southeast University

## 原题目要求
**基本要求：** 
利用板载陀螺仪实现平衡车自动控制


**发挥部分：**
为平衡车添加一些其它的基本功能，如自动巡线，红绿灯检测等
功能

## 初步计划

本作品设计一台两轮平衡小车，利用板载陀螺仪、摄像头、超声波、麦克风模块，链接手机APP，组成一台有趣、自主的平衡车。 
1. **自平衡**：小车通过板载陀螺仪获取当前平衡小车的姿态，通过PID算法对平衡小车的姿态进行实时调整，实现小车在静止、行进中的自平衡。 
2. **循线**：小车通过摄像头回传的RGB色块实现检测识别路线，从而沿线路自主运行，运用PID算法保证不偏离路线的同时尽可能快地行进。
3. **红绿检测** 在运行的过程中，小车将检测红绿灯，并相应地停止或前进。
4. **防坠 和 避障**：小车通过超声波模块，判断前方的路面情况进，当行进至“悬崖”或者障碍前时自动停止，并向手机APP传送警报信息。 
5. **无线**：手机通过ESP32的WIFI/Bluetooth模块连接小车，能够显示当前小车参数。用户能够在APP上选择小车进入 循线、跟随 或遥控模式。 循线模式下小车将找到最近的一条引导线，进行循线； 跟随模式下小车将跟随手机用户行进。 遥控模式下小车将听从用户的遥控操作，用户可以通过手机的手势（陀螺仪）或按键遥控小车进行前进、后退、转向、花式动作等。 
本小车对于家庭娱乐、社会服务等行业发展可能可以起到试验和探索作用。

## 初步分工

- 张逸帆：循线
- 魏晨阳，刘昊东：自平衡

## 参考资料
[地址](https://www.yahboom.com/study/bc-32)

[SEA Demo](https://github.com/sea-s7/Demo_project)

[陀螺仪Arduino](https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library)

[板载陀螺仪](https://github.com/sea-s7/Demo_project/tree/master/On-Board-Gyroscope)

[PID on FPGA](https://github.com/sea-s7/Demo_project/tree/master/PID-Algorithm-On-FPGA)

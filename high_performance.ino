#define I2CAddress 0x6b
#include "Wire.h"

uint8_t nonSuccessCounter = 0;
float ax,ay,az,gx,gy,gz = 0;

uint8_t ReadRegister(uint8_t * outputPointer, uint8_t offset)
{
    uint8_t result = 0;
    uint8_t numBytes = 1;
    uint8_t returnError = 0;
    Wire.beginTransmission(I2CAddress);
    Wire.write(offset);
    if( Wire.endTransmission() != 0 )
    {
      returnError = 1;
    }
    Wire.requestFrom(I2CAddress, numBytes);
    while ( Wire.available() ) // slave may send less than requested
    {
      result = Wire.read(); // receive a byte as a proper uint8_t
    }
    *outputPointer = result;
    return returnError;
}

uint8_t BeginCore(){
  uint8_t returnError = 0;
  Wire.begin();
  delay(10);
  uint8_t readCheck;
  ReadRegister(& readCheck, 0x0f);
  if( readCheck != 0x69 )
  {
    returnError = 1;
  }
  return returnError;
}

uint8_t WriteRegister(uint8_t offset, uint8_t dataToWrite){
  uint8_t returnError = 0;
  Wire.beginTransmission(I2CAddress);
  Wire.write(offset);
  Wire.write(dataToWrite);
  if( Wire.endTransmission() != 0 )
  {
    returnError = 1;
  }
  return returnError;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  uint8_t dataToWrite = 0;
  uint8_t returnError = BeginCore();
  dataToWrite = 0x03 | 0x00 | 0x40;//带宽50Hz，范围2g，104Hz采样率
  WriteRegister(0x10, dataToWrite);
//  ReadRegister(&dataToWrite, 0x13);
//  dataToWrite &= ~((uint8_t)0x80);
  /*if ( settings.accelODROff == 1) {
    dataToWrite |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
  }*/
  WriteRegister(0x13, 0x08);
  dataToWrite = 0x04 | 0x40  ;//125dps ,104Hz采样率
  WriteRegister(0x11, dataToWrite);
  //自己加的
  ReadRegister(&dataToWrite, 0x15);
  dataToWrite &= 0xe0;
  WriteRegister(0x15, dataToWrite);//高性能开启
  ReadRegister(&dataToWrite, 0x16);
  dataToWrite &= 0x7f;
  WriteRegister(0x16, WriteRegister);
}

uint8_t ReadRegisterInt16( int16_t* outputPointer, uint8_t offset ){
  uint8_t myBuffer[2];
  uint8_t returnError = ReadRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
  int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
  
  *outputPointer = output;
  return returnError;
}

uint8_t ReadRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t len){
  uint8_t returnError = 0;
  uint8_t i = 0;
  uint8_t c = 0;
  uint8_t tempFFCounter = 0;
  Wire.beginTransmission(I2CAddress);
    Wire.write(offset);
    if( Wire.endTransmission() != 0 )
    {
      returnError = 1;
    }
    else  //OK, all worked, keep going
    {
      // request 6 bytes from slave device
      Wire.requestFrom(I2CAddress, len);
      while ( (Wire.available()) && (i < len))  // slave may send less than requested
      {
        c = Wire.read(); // receive a byte as character
        *outputPointer = c;
        outputPointer++;
        i++;
      }
    }
  return returnError;
}

void ReadFloatAccel(float &x,float &y,float & z){
  int16_t output_x,output_y,output_z;
  int8_t errorLevel = ReadRegisterInt16( &output_x, 0x28 );
  ReadRegisterInt16( &output_y, 0x2a );
  ReadRegisterInt16( &output_z, 0x2c );
  
  if( errorLevel != 0 )
  {
      ++nonSuccessCounter;
      Serial.print("errorCount:");
      Serial.println(nonSuccessCounter);
  }
  
    x = (float)output_x * 0.061 * ( 2 >> 1) / 1000;//注意这里range要变
    y = (float)output_y * 0.061 * ( 2 >> 1) / 1000;//注意这里range要变
    z = (float)output_z * 0.061 * ( 2 >> 1) / 1000;//注意这里range要变
  
}
//0x24
void ReadFloatGyro(float &x,float &y,float & z){
  int16_t output_x,output_y,output_z;
  int8_t errorLevel = ReadRegisterInt16( &output_x, 0x22 );
  ReadRegisterInt16( &output_y, 0x24 );
  ReadRegisterInt16( &output_z, 0x26 );

  if( errorLevel != 0 )
  {
      ++nonSuccessCounter;
      Serial.print("errorCount:");
      Serial.println(nonSuccessCounter);
  }

  uint8_t gyroRangeDivisor = 500 / 125;//这里500值500dps
  x = (float)output_x * 4.375 * (gyroRangeDivisor) / 1000;
  y = (float)output_y * 4.375 * (gyroRangeDivisor) / 1000;
  z = (float)output_z * 4.375 * (gyroRangeDivisor) / 1000;
}


//**************************************************卡尔曼滤波参数与函数  

float dt=0.001;//注意：dt的取值为kalman滤波器采样时间  
float angle, angle_dot;//角度和角速度  
float P[2][2] = {{ 1, 0 },  
                 { 0, 1 }};  
float Pdot[4] ={ 0,0,0,0};  
float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度,角速度数据置信度  
float R_angle=0.5 ,C_0 = 1;  
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;  
//卡尔曼滤波  

//卡尔曼滤波  
float Kalman_Filter(float angle_m, float gyro_m)//angleAx 和 gyroGy   
{   
    float dt=0.001;//注意：dt的取值为kalman滤波器采样时间  
    float angle, angle_dot;//角度和角速度  
    float P[2][2] = {{ 1, 0 },  
                 { 0, 1 }};  
    float Pdot[4] ={ 0,0,0,0};  
    float Q_angle=0.001, Q_gyro=0.005; //角度数据置信度,角速度数据置信度  
    float R_angle=0.5 ,C_0 = 1;  
    float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;  

          
  angle+=(gyro_m-q_bias) * dt;          
  angle_err = angle_m - angle;          
  Pdot[0]=Q_angle - P[0][1] - P[1][0];          
  Pdot[1]=- P[1][1];          
  Pdot[2]=- P[1][1];          
  Pdot[3]=Q_gyro;          
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
  angle += K_0 * angle_err; //最优角度          
  q_bias += K_1 * angle_err;          
  angle_dot = gyro_m-q_bias;//最优角速度             
  return angle;  
}  




/************************************循环*****************************************************/
void loop() {
  // put your main code here, to run repeatedly:
    float angleAy = 0,angle = 0;
    ReadFloatAccel(ax,ay,az);
    ReadFloatGyro(gx,gy,gz);
//    Serial.println(ax);
//    Serial.println(ay);
//    Serial.println(az);
//    Serial.println(gx);
//    Serial.println(gy);
//    Serial.println(gz);
    angleAy = atan2(ay,az)*180/PI;
    angle = Kalman_Filter(angleAy,gy);
    Serial.println(angle);
    delay(20);

    
}

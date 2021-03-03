
#include <Wire.h>


const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float gyroX, gyroY, gyroZ;
float temp1, temp2;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

void setup(){

  

 Serial.begin(19200);

 Wire.begin();

 // Make Communication and Reset
 Wire.beginTransmission(MPU);
 Wire.write(0x6B);
 Wire.write(0x00);
 Wire.endTransmission(true);

 // Accelerometer Sensitivity
 Wire.beginTransmission(MPU);
 Wire.write(0x1C);
 Wire.write(0x00); // 2g
 Wire.endTransmission(true);
 
 // Gyro Sensitivity
 Wire.beginTransmission(MPU);
 Wire.write(0x1b);
 Wire.write(0x10);
 Wire.endTransmission(true);

 
 calculate_IMU_error(200);

 
}


void loop ()

{

  previousTime = currentTime;   
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  
  float * data = get_data();

  
  AccX  = data[0];
  AccY  = data[1];
  AccZ  = data[2];
  
  GyroX = data[3] - gyroErrorX;
  GyroY = data[4] - gyroErrorY;
  GyroZ = data[5] - gyroErrorZ;
  

  accAngleX = AccelerometerX(AccX, AccY, AccZ) - accErrorX;
  accAngleY = AccelerometerY(AccX, AccY, AccZ) - accErrorY;


  gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  
  yaw =  yaw + GyroZ * elapsedTime;
  
  roll  = 0.90 * gyroAngleX + 0.1 * accAngleX;
  pitch = 0.90 * gyroAngleY + 0.1 * accAngleY;
  
  
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print(roll);
  Serial.println("\t");
  

  
  
}

float* get_data(){

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  static float data[6];


  data[0] = (Wire.read() << 8 | Wire.read()) / 16384.0;
  data[1] = (Wire.read() << 8 | Wire.read()) / 16384.0;
  data[2] = (Wire.read() << 8 | Wire.read()) / 16384.0;

  temp1   = (Wire.read() << 8 | Wire.read());

  data[3] = (Wire.read() << 8 | Wire.read()) / 500.0;
  data[4] = (Wire.read() << 8 | Wire.read()) / 500.0;
  data[5] = (Wire.read() << 8 | Wire.read()) / 500.0;


  return data;
  
  }


void calculate_IMU_error(int cycles){
  
  for (int i = 0; i <= cycles; i++){

    float * data = get_data();

    AccX   = data[0];
    AccY   = data[1];
    AccZ   = data[2];
    gyroX  = data[3];
    gyroY  = data[4];
    gyroZ  = data[5];

    AccErrorX = AccErrorX + AccelerometerX(AccX, AccY, AccZ);
    AccErrorY = AccErrorY + AccelerometerY(AccX, AccY, AccZ);

    GyroErrorX = GyroErrorX + gyroX;
    GyroErrorY = GyroErrorY + gyroY;
    GyroErrorZ = GyroErrorZ + gyroZ;

  };


  accErrorX = AccErrorX  / cycles;
  accErrorY = AccErrorY  / cycles;
  gyroErrorX = GyroErrorX / cycles;
  gyroErrorY = GyroErrorY / cycles;
  gyroErrorZ = GyroErrorZ / cycles;

  
  }

float AccelerometerX(float accX, float accY, float accZ){
  return (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))));
  }


float AccelerometerY(float accX, float accY, float accZ){
  return(atan(AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))));
  }

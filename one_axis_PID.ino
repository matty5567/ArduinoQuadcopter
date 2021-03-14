
/*
 * One axis flight controller using PID control to stabilise roll
 * 
 */


#include <Servo.h>
#include <Wire.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN_1 9
#define MOTOR_PIN_2 10

#define k1_p 1
#define k2_i 0.005
#define k3_i 0 

#define RAD_TO_DEG 57.29577

const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float gyroX, gyroY, gyroZ;
float temp1;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float accErrorX, accErrorY, gyroErrorX, gyroErrorY, gyroErrorZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
int DELAY = 1000;

float displacement;
float integral;

Servo motor_1;
Servo motor_2;

void setup() {
 Serial.begin(9600);

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
 Wire.write(0x10); // +- 1000 deg/s
 Wire.endTransmission(true);

 Serial.println("Calibrating gyro and accelerometer");
 calculate_IMU_error(1000);
 Serial.println("finished calibration");
  
  
  motor_1.attach(MOTOR_PIN_1);
  motor_2.attach(MOTOR_PIN_2);
  
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  motor_1.writeMicroseconds(MIN_SIGNAL);
  motor_2.writeMicroseconds(MIN_SIGNAL);
  

  // Wait for input
  while (!Serial.available());
  Serial.read();

  // Send min output
  
  motor_1.writeMicroseconds(MIN_SIGNAL);
  motor_2.writeMicroseconds(MIN_SIGNAL);

    // Wait for input
  while (!Serial.available());
  Serial.read();
  currentTime = millis();
  Serial.println("Finished setup loop");
  
}

void loop(){  

  previousTime = currentTime;   
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  
  float * data = get_data();

  
  AccX  = data[0];
  AccY  = data[1];
  AccZ  = data[2];
  
  GyroY = data[4] - 0.7;
  

  gyroAngleY = pitch + GyroY * elapsedTime;
  //accAngleX = AccelerometerX(AccX, AccY, AccZ) - accErrorX;

  accAngleX = AccelerometerX(AccX, AccY, AccZ);
  
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleX;

  displacement = displacement + pitch;
  integral = integral + displacement * elapsedTime;


  int average_width = 1200;
 
  int m_2_width = average_width + k1_p * pitch + k2_i * displacement + k3_i * integral;
  int m_1_width = average_width - k1_p * pitch - k2_i * displacement + k3_i * integral;

  motor_1.writeMicroseconds(m_1_width);
  motor_2.writeMicroseconds(m_2_width);

  
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(gyroAngleY);
  Serial.print("\t");
  Serial.print(accAngleX);
  Serial.print("\t");
  Serial.print(i_pitch);
  Serial.print("\t");
  Serial.print(m_2_width);
  Serial.print("\t");
  Serial.println(m_1_width);
                                         
  
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


  accErrorY  = AccErrorY  / cycles;
  gyroErrorX = GyroErrorX / cycles;
  gyroErrorY = GyroErrorY / cycles;
  gyroErrorZ = GyroErrorZ / cycles;

  Serial.println(gyroErrorY);

  
  }

float AccelerometerX(float accX, float accY, float accZ){
  return (-atan(accX / (sqrt(pow(accY, 2) + pow(accZ, 2)))) * RAD_TO_DEG);
  }


float AccelerometerY(float accX, float accY, float accZ){
  
  return (-atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * RAD_TO_DEG);
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

  data[3] = (Wire.read() << 8 | Wire.read())/32.8;
  data[4] = (Wire.read() << 8 | Wire.read())/32.8;
  data[5] = (Wire.read() << 8 | Wire.read())/32.8;


  return data;
  
  }
 

#include <Servo.h>
#include <Wire.h>

#define MOTOR_PIN_1 12
#define MOTOR_PIN_2 10
#define MOTOR_PIN_3 9
#define MOTOR_PIN_4 11
#define MIN_SIGNAL 1000
#define pitchSensitivity 1
#define rollSensitivity 1

unsigned int reciever[6];
unsigned int currTime, prevTime;
int throttle_control, pitch_control, roll_control, yaw_control, rightSwitch, leftSwitch;
int frontLeftSend, frontRightSend, backLeftSend, backRightSend;
int counter, width;
bool state, on;

float mpu_data[6];
const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;

float temp1, temp2;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float accErrorX, accErrorY, accErrorZ, gyroErrorX, gyroErrorY, gyroErrorZ;
float sumAccErrorX, sumAccErrorY, sumAccErrorZ, sumGyroErrorX, sumGyroErrorY, sumGyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
int roll_integral, pitch_integral;
float roll_adj, pitch_adj;

////////////////////////////////////////////////
// Define Gains
#define roll_P_gain 1.3
#define roll_I_gain 0
#define roll_D_gain 0

#define pitch_P_gain 1.3
#define pitch_I_gain 0
#define pitch_D_gain 0

////////////////////////////////////////////////


Servo frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  attachInterrupt(7, readPPM, CHANGE);
    
  frontLeftMotor.attach(MOTOR_PIN_1);
  frontRightMotor.attach(MOTOR_PIN_2);
  backLeftMotor.attach(MOTOR_PIN_3);
  backRightMotor.attach(MOTOR_PIN_4);
  

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

    frontLeftMotor.writeMicroseconds(MIN_SIGNAL);
    frontRightMotor.writeMicroseconds(MIN_SIGNAL);
    backLeftMotor.writeMicroseconds(MIN_SIGNAL);
    backRightMotor.writeMicroseconds(MIN_SIGNAL); 

   Serial.print("Start calibration");
   calculate_IMU_error(200);
   Serial.print("End calibration");

}


void loop() {
  
  
  previousTime = currentTime;   
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) * 0.0001 ;
  
  read_mpu();
  
  accAngleX = AccelerometerX(AccX-accErrorX, AccY-accErrorY, AccZ-accErrorZ);
  accAngleY = AccelerometerY(AccX, AccY, AccZ);


  gyroAngleX = pitch + (GyroX - gyroErrorX) * elapsedTime;
  gyroAngleY = roll + (GyroY - gyroErrorY) * elapsedTime;
  
  yaw =  yaw + GyroZ * elapsedTime;
  
  pitch  = 0.96 * gyroAngleX + 0.04 * accAngleX;
  roll = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  on = false;
  
  if (rightSwitch < 1050 & rightSwitch > 950){
    on = true;
    }

  
  roll_control = reciever[0]; //Channel1 : Right joystick Left/ Right(Roll)
  pitch_control = reciever[1]; //Right joystick Up /Down (Pitch)
  throttle_control = reciever[2]; //Left joystick Up/ Down (Throttle)
  yaw_control = reciever[3]; //left joystick Left/Right (Yaw)
  leftSwitch = reciever[4]; //Left Switch
  rightSwitch = reciever[5]; //Right Switch

  
  for (int i=0; i<6; i++){
    Serial.print(reciever[i]);
    Serial.print('\t');
    }

    Serial.print('\t');

  
  

  roll_integral += roll;
  pitch_integral += pitch;

  roll_adj = roll_P_gain * roll + roll_D_gain * GyroX + roll_I_gain * roll_integral;
  pitch_adj = pitch_P_gain * pitch + pitch_D_gain * GyroX + pitch_I_gain * pitch_integral;

//  Serial.print(roll);
//  Serial.print('\t');
//  Serial.println(pitch);

  
  
  
  if (on == true){

      frontLeftSend = throttle_control + pitchSensitivity *(1500 - pitch_control) - pitch_P_gain * pitch  + rollSensitivity * (roll_control - 1500) + roll_P_gain * roll;
      frontRightSend = throttle_control + pitchSensitivity * (1500 - pitch_control) - pitch_P_gain * pitch + rollSensitivity * (1500 - roll_control) - roll_P_gain * roll;
      backLeftSend = throttle_control + pitchSensitivity * (pitch_control - 1500) + pitch_P_gain * pitch + rollSensitivity * (roll_control - 1500) + roll_P_gain * roll;
      backRightSend = throttle_control + pitchSensitivity * (pitch_control - 1500) + pitch_P_gain * pitch + rollSensitivity * (1500 - roll_control) - roll_P_gain * roll;
   }
   
  else{
      frontLeftSend  = MIN_SIGNAL;
      frontRightSend = MIN_SIGNAL;
      backLeftSend   = MIN_SIGNAL;
      backRightSend  = MIN_SIGNAL;
    }

      frontLeftMotor.writeMicroseconds(frontLeftSend);
      frontRightMotor.writeMicroseconds(frontRightSend);
      backLeftMotor.writeMicroseconds(backLeftSend);
      backRightMotor.writeMicroseconds(backRightSend); 


      Serial.print(frontLeftSend);
      Serial.print('\t');
      Serial.print(frontRightSend);
      Serial.print('\t');
      Serial.print(backLeftSend);
      Serial.print('\t');
      Serial.println(backRightSend);
      
}



void readPPM(){
  state = digitalRead(7);
  if (state==1){
    prevTime = micros();
    return;
    }
    
  currTime = micros();
  width = currTime - prevTime;

  if (width > 2000){
    counter = 0;
    return;
    }
    
  reciever[counter] = width + 400;
  counter ++;
  
  }




void read_mpu(){

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);


  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;

  temp1   = (Wire.read() << 8 | Wire.read());

  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  }


void PID_hover(){

  }

float AccelerometerX(float accX, float accY, float accZ){
  return (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
  }


float AccelerometerY(float accX, float accY, float accZ){
  return(atan(AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
  }

void calculate_IMU_error(int cycles){
  
  for (int i = 0; i <= cycles; i++){

    read_mpu();
    

    sumGyroErrorX = sumGyroErrorX + GyroX;
    sumGyroErrorY = sumGyroErrorY + GyroY;
    sumGyroErrorZ = sumGyroErrorZ + GyroZ;

    sumAccErrorX += AccX;
    sumAccErrorY += AccY;
    sumAccErrorZ += AccZ;

  };

  gyroErrorX = sumGyroErrorX / cycles;
  gyroErrorY = sumGyroErrorY / cycles;
  gyroErrorZ = sumGyroErrorZ / cycles;

  accErrorX = sumAccErrorX / cycles;
  accErrorY = sumAccErrorY / cycles;
  accErrorZ = sumAccErrorZ / cycles;

  
  }

#include <Servo.h>

#define MOTOR_PIN_1 12
#define MOTOR_PIN_2 10
#define MOTOR_PIN_3 9
#define MOTOR_PIN_4 11
#define MIN_SIGNAL 900
#define pitchSensitivity 0.1
#define rollSensitivity 0.1

unsigned int reciever[6];
unsigned int currTime, prevTime;
int throttle, pitch, roll, yaw, rightSwitch, leftSwitch;
int frontLeftSend, frontRightSend, backLeftSend, backRightSend;
int counter, width;
bool state, on;

Servo frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

void setup() {
  Serial.begin(9600);
  attachInterrupt(7, readPPM, CHANGE);
  


  
  frontLeftMotor.attach(MOTOR_PIN_1);
  frontRightMotor.attach(MOTOR_PIN_2);
  backLeftMotor.attach(MOTOR_PIN_3);
  backRightMotor.attach(MOTOR_PIN_4);
  

//
//   delay(3000);
//
//  frontLeftMotor.writeMicroseconds(1000);
//  frontRightMotor.writeMicroseconds(1000);
//  backLeftMotor.writeMicroseconds(1000);
//  backRightMotor.writeMicroseconds(1000);
//  

  //Serial.println("Turn on power source, then press any key.");

  
  //frontLeftMotor.writeMicroseconds(MIN_SIGNAL);
  //frontRightMotor.writeMicroseconds(MIN_SIGNAL);
  //backLeftMotor.writeMicroseconds(MIN_SIGNAL);
  //backRightMotor.writeMicroseconds(MIN_SIGNAL); 

  // Wait for input
  //while (!Serial.available());
  //Serial.read();

  // Send min output
  
  //frontLeftMotor.writeMicroseconds(MIN_SIGNAL);
  //frontRightMotor.writeMicroseconds(MIN_SIGNAL);
  //backLeftMotor.writeMicroseconds(MIN_SIGNAL);
  //backRightMotor.writeMicroseconds(MIN_SIGNAL); 
  
  //Serial.println("Finished setup loop");

}

void loop() {
  //Serial.print(on);
  //Serial.print('\t');
  //Serial.println(rightSwitch);
  
  on = false;
  if (rightSwitch < 1750){
    on = true;
    }

  
  //Serial.print(rightSwitch);
  //Serial.print('\t');
  //Serial.println(on);
  
  
  
  roll = reciever[0]; //Channel1 : Right joystick Left/ Right(Roll)
  pitch = reciever[1]; //Right joystick Up /Down (Pitch)
  throttle = reciever[2]; //Left joystick Up/ Down (Throttle)
  yaw = reciever[3]; //left joystick Left/Right (Yaw)
  leftSwitch = reciever[4]; //Left Switch
  rightSwitch = reciever[5]; //Right Switch

  /*
  for (int i=0; i<6; i++){
    Serial.print(reciever[i]);
    Serial.print('\t');
    }
   Serial.println();
  
  Serial.print(frontLeftSend);
  Serial.print('\t');
  Serial.print(frontRightSend);
  Serial.print('\t');
  Serial.print(backLeftSend);
  Serial.print('\t');
  Serial.println(backRightSend);
  */
  frontLeftSend = throttle + pitchSensitivity *(1500 - pitch) + rollSensitivity * (roll - 1500);
  frontRightSend = throttle + pitchSensitivity * (1500 - pitch) + rollSensitivity * (1500 - roll);
  backLeftSend = throttle + pitchSensitivity * (pitch - 1500) + rollSensitivity * (roll - 1500);
  backRightSend = throttle + pitchSensitivity * (pitch - 1500) + rollSensitivity * (1500 - roll);
  
  if (on == true){

      frontLeftMotor.writeMicroseconds(throttle);
      frontRightMotor.writeMicroseconds(throttle);
      backLeftMotor.writeMicroseconds(throttle);
      backRightMotor.writeMicroseconds(throttle); 
}
  else{
      frontLeftMotor.writeMicroseconds(MIN_SIGNAL);
      frontRightMotor.writeMicroseconds(MIN_SIGNAL);
      backLeftMotor.writeMicroseconds(MIN_SIGNAL);
      backRightMotor.writeMicroseconds(MIN_SIGNAL); 
    }
}




void readPPM(){
  state = digitalRead(7);


  if (state==1){
    prevTime = micros();
//    counter ++;
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

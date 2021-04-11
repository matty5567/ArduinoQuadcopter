
int throttle, yaw, pitch, roll;
int throttleStart, yawStart, rollStart, pitchStart;
int pulse = 0;
int curTime;



const byte interruptPin1 = 2;
const byte interruptPin2 = 3;
const byte interruptPin3 = 7;
const byte interruptPin4 = 6;




void setup() {
  Serial.begin(57600);
  attachInterrupt(2, readThrottle, CHANGE);
  attachInterrupt(3, readYaw, CHANGE);
  attachInterrupt(6, readPitch, CHANGE);
  attachInterrupt(7, readRoll, CHANGE);
}

void loop() {
  
  Serial.print(throttle);
  Serial.print('\t');
  Serial.print(yaw);
  Serial.print('\t');
  Serial.print(pitch);
  Serial.print('\t');
  Serial.println(roll);
  
}

void readThrottle(){
  curTime = micros();
  if (digitalRead(2) == 1){
    throttleStart = curTime;
    return;
    };
  throttle = curTime - throttleStart;
  }

void readYaw(){
  curTime = micros();
  if (digitalRead(3) == 1){
    yawStart = curTime;
    return;
    };
  yaw = curTime - yawStart;
  }

void readPitch(){
  curTime = micros();
  if (digitalRead(6) == 1){
    pitchStart = curTime;
    return;
    };
  pitch = curTime - pitchStart;
  }

void readRoll(){
  curTime = micros();
  if (digitalRead(7) == 1){
    rollStart = curTime;
    return;
    };
  roll = curTime - rollStart;

  }

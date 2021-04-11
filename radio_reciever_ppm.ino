unsigned int currTime, prevTime;
int counter, width;
bool state;

void setup() {
  Serial.begin(57600);
  attachInterrupt(7, readPPM, CHANGE);


}

void loop() {
  for (int i=0; i<6; i++){
    Serial.print(reciever[i]);
    Serial.print('\t');
    }
    Serial.println();

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

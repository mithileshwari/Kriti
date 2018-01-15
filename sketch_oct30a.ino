#include <Servo.h>

Servo servo0;
Servo servo1;

// create array
int incoming[2];

void setup(){
  Serial.begin(9600);

  servo0.attach(3);
  servo1.attach(6);
}

void loop(){
  while(Serial.available() >= 2){
    // fill array
    for (int i = 0; i < 2; i++){
      incoming[i] = Serial.read();
    }
    // use the values
    servo0.write(incoming[0]);
    servo1.write(incoming[1]);
  }
}

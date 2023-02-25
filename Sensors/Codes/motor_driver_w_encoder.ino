#include <Encoder.h>

#define PWM1 8
#define AIN1 10
#define AIN2 9

Encoder myEnc(5, 6);

long oldPosition  = -999;

int out = 255;
int spd = 100;

void setup() {
  Serial.begin(9600);
  pinMode(PWM1,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
}
 
void loop() {
  if(spd >= 0) {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
  }
  else {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
  }
  
  analogWrite(PWM1,out);

  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.print(millis() / 1000);
    Serial.print(": ");
    Serial.println(newPosition / 4 / 360);
  }
}

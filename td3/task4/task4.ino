#include "Arduino.h"
#include "MotorDriver.h"
#include "Ticker.h"
#include "ESP32Encoder.h"

const int signalPin = 14; 

long duration;
float distance;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  pinMode (signalPin, OUTPUT);
  digitalWrite(signalPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(signalPin, LOW);
  pinMode (signalPin, INPUT);
  delayMicroseconds(10);
  duration = pulseIn(signalPin, HIGH);
  distance= duration*0.034/2;
  Serial.print("Distance: ");
  Serial.println(distance);

}

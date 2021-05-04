//used for testing libraries
#include "Arduino.h"
#include "buggy.h"
#include "Ticker.h"

const int dt_millis = 50;
float dt = dt_millis/1000.0;
float sonarDist;

Motor lmotor(2, 2, 12, 0, 39, 35, dt_millis);
Ticker updateMotorSpeeds; //could not be called inside class, callback() does not exist as in Mbed


void adjustSpeeds() //ISR
{
  lmotor.adjustSpeed();
}

void setup()
{
  Serial.begin(9600);
  updateMotorSpeeds.attach_ms(dt_millis, &adjustSpeeds);
  lmotor.setAngularSpeed(2);
}

void loop()
{
  
}

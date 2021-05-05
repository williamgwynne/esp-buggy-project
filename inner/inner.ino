//used for testing libraries
#include "Arduino.h"
#include "buggy.h"
#include "Ticker.h"

const int dt_millis = 100;
float dt = dt_millis/1000.0;
float sonarDist;

Ticker updateMotorSpeeds; //could not be called inside class, callback() does not exist as in Mbed
Buggy buggy(dt_millis);

void getSonarDist() //ISR
{
  sonarDist = buggy.sonar.getDist();
}

void adjustSpeeds() //ISR
{
  buggy.adjustSpeeds();
}


void setup()
{
  Serial.begin(9600);
  updateMotorSpeeds.attach_ms(dt_millis, &adjustSpeeds);
  buggy.servo.write(90);
}

float errorDistance_prev = 0;
float time_still=0;

void loop()
{
  for (float i=1; i<14; i+=0.5)
  {
    buggy.left_motor.setAngularSpeed(i);
    buggy.right_motor.setAngularSpeed(i);  
    delay(1000);
  }
}

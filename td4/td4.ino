//used for testing libraries
#include "Arduino.h"
#include "buggy.h"
#include "Ticker.h"

const int dt_millis = 100;
float sonarDist;

Motor right_motor(4, 2, 15, 18, 34, 36, dt_millis);
Motor left_motor(2, 1, 12, 0, 39, 35, dt_millis);
Ticker updateMotorSpeeds; //could not be called inside class, callback() does not exist as in Mbed
Ticker sendPulses; //same as ^^

Sonar sonar(14);

Servo servo; //(5);


void getSonarDist() //ISR
{
  sonarDist = sonar.getDist();
}

void adjustSpeeds() //ISR
{
  right_motor.adjustSpeed();
  left_motor.adjustSpeed();
}

void setup()
{
  Serial.begin(9600);
  updateMotorSpeeds.attach_ms(dt_millis, &adjustSpeeds);
  //sendPulses.attach_ms(dt_millis, &getSonarDist); //only uncommmented so LED didn't annoy me
  right_motor.setAngularSpeed(0); //max speed = ~14.93 rads/s
  left_motor.setAngularSpeed(0);
}

void loop()
{
  servo.write(90);
}

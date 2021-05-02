//used for testing libraries
#include "Arduino.h"
#include "buggy.h"
#include "Ticker.h"

const int dt_millis = 50;
int dt = dt_millis/1000.0;
float sonarDist;

Ticker updateMotorSpeeds; //could not be called inside class, callback() does not exist as in Mbed
Ticker sendPulses; //same as ^^
Ticker lineFollow;
Buggy buggy(dt_millis);

void getSonarDist() //ISR
{
  sonarDist = buggy.sonar.getDist();
}

void adjustSpeeds() //ISR
{
  buggy.adjustSpeeds();
}

void followLine() //ISR
{
  buggy.followLine();
}

void setup()
{
  Serial.begin(9600);
  updateMotorSpeeds.attach_ms(dt_millis, &adjustSpeeds);
  //sendPulses.attach_ms(dt_millis, &getSonarDist); //only uncommmented so LED didn't annoy me
  lineFollow.attach_ms(dt_millis, &followLine);
//  buggy.right_motor.setAngularSpeed(0); //max speed = ~14.93 rads/s
//  buggy.left_motor.setAngularSpeed(0);
  buggy.servo.write(90);
}

void loop()
{


  delay(100); //Delay before next loop iteration
}

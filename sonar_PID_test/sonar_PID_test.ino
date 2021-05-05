//used for testing libraries
#include "Arduino.h"
#include "buggy.h"
#include "Ticker.h"

const int dt_millis = 50; //keep at 50 ms
float dt = dt_millis/1000.0;
float sonarDist;

Ticker updateMotorSpeeds; //could not be called inside class, callback() does not exist as in Mbed
Ticker sendPulses; //same as ^^
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
  sendPulses.attach_ms(dt_millis, &getSonarDist); //only uncommmented so LED didn't annoy me
  buggy.servo.write(90);
}

float errorDistance_prev = 0;
float time_still=0;

void loop()
{
  float kp_dist = 0.0015; //tuned for max. angular speed of 9 rads/sec. To change speed, P values needs changing
  
  float errorDistance = sonarDist - 10; //steady state of 25cm

  float w_desired = ((errorDistance * kp_dist)/dt)/0.05;



  buggy.left_motor.setAngularSpeed(w_desired);
  buggy.right_motor.setAngularSpeed(w_desired);

  errorDistance_prev = errorDistance;
  Serial.println(w_desired);
  delay(dt_millis);
  
}

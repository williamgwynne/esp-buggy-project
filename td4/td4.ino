//used for testing libraries
#include "Arduino.h"
#include "buggy.h"
#include "Ticker.h"

const int dt_millis = 50;
float dt = dt_millis/1000.0;
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

void turnAround()
{
  lineFollow.detach(); //stops following the line whilst the buggy turns around
  buggy.left_motor.setAngularSpeed(5);
  buggy.right_motor.setAngularSpeed(-5);
  delay(1800);
  lineFollow.attach_ms(dt_millis, &followLine);
}

void setup()
{
  Serial.begin(9600);
  updateMotorSpeeds.attach_ms(dt_millis, &adjustSpeeds);
  sendPulses.attach_ms(dt_millis, &getSonarDist); //only uncommmented so LED didn't annoy me
  lineFollow.attach_ms(dt_millis, &followLine);
  buggy.servo.write(90);
}

float errorDistance_prev = 0;
float time_still=0;

void loop()
{
  float kp_dist = 0.05, kd_dist=0;
  
  float errorDistance = sonarDist - 30; //error of 30cm
  
  float proportionalDist = errorDistance * kp_dist;
  float differentialDist = kd_dist*(errorDistance - errorDistance_prev)/dt;
  float w_desired = ((proportionalDist + differentialDist)/dt)/0.05;
  
  if (w_desired>10)
  {
    time_still=0;
    w_desired = 10;
  }
  else if (w_desired<0)
  {
    w_desired = 0;
    if (time_still > 500) //after 500ms staying still, buggy turns round, reduces error
      turnAround();
    time_still+=dt_millis;
  }
  errorDistance_prev = errorDistance;
  buggy.w_desired = w_desired;
  delay(dt_millis);
}

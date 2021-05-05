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
  delay(1300);
  lineFollow.attach_ms(dt_millis, &followLine);
}

void stopRunning()
{
  updateMotorSpeeds.detach();
  sendPulses.detach();
  lineFollow.detach();
  delay(dt_millis);
  buggy.left_motor.stop_();
  buggy.right_motor.stop_();
  while (true)
  {
    //infinite loop
  }
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
  float kp_dist = 0.0025; //tuned for max. angular speed of 9 rads/sec, stopping distance of 10cm. To change speed, P values need changing
  
  float errorDistance = sonarDist - 20; //steady state of 0cm


  //Proportional-only ontroller as uncertainty from suddenly changing error values can cause serious issues
  float w_desired = ((errorDistance * kp_dist)/dt)/0.05;

  if(buggy.stop_)
    stopRunning();
  
  if (w_desired>9) //9 is a cautionary speed, set a higher one for 2nd submission
  {
    time_still=0;
    w_desired = 9;
  }
  else if (w_desired<1.5) //trapping slow speeds
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

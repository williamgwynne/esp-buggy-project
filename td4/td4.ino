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
  if (millis() > 5000) //stops accidental turnaround at beginning
  {
    lineFollow.detach(); //stops following the line whilst the buggy turns around
    buggy.left_motor.setAngularSpeed(5);
    buggy.right_motor.setAngularSpeed(-5);
    delay(1500);
    lineFollow.attach_ms(dt_millis, &followLine);
  }
}

void stopRunning()
{
  updateMotorSpeeds.detach();
  sendPulses.detach();
  lineFollow.detach();
  delay(dt_millis);
  buggy.left_motor.stop_();
  buggy.right_motor.stop_();
  int tol = 0;
  int flag_lineSensor = 0;
  uint32_t sensorVals[7];
  while (tol<500)
  {
    flag_lineSensor = 0;
    buggy.lineSensor.ReadSensor();
    for (int i = 0; i < 7; i++)
      {
        sensorVals[i] = buggy.lineSensor.GetSensorValues(i);
        if (sensorVals[i]<1700)
          flag_lineSensor = 1;
      }
    if (flag_lineSensor)
      tol+=dt_millis;
    else
      tol=0;
    delay(dt_millis);
  }
  updateMotorSpeeds.attach_ms(dt_millis, &adjustSpeeds); //returns to original code if put back on line
  sendPulses.attach_ms(dt_millis, &getSonarDist); //only uncommmented so LED didn't annoy me
  lineFollow.attach_ms(dt_millis, &followLine);
  buggy.stop_ = 0;
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
  float kp_dist = 0.001; //tuned for max. angular speed of 8 rads/sec, stopping distance of 10cm. To change speed, P values need changing
  float w_desired;
  if(buggy.stop_)
    stopRunning();
  if (sonarDist > 0) //to cancel out error where no value is returned
  {
    float errorDistance = sonarDist - 15; //steady state of 15cm
    //Proportional-only ontroller as uncertainty from suddenly changing error values can cause serious issues
    w_desired = ((errorDistance * kp_dist)/dt)/0.05;
    
    if (w_desired>8) //9 is a cautionary speed, set a higher one for 2nd submission
    {
      time_still=0;
      w_desired = 8;
    }
    else if (w_desired<1.5) //trapping slow speeds
    {
      if (w_desired<0)
        w_desired = 0;
      if (time_still > 500) //after 500ms staying still, buggy turns round, reduces error
        turnAround();
      time_still+=dt_millis;
    }
    errorDistance_prev = errorDistance;
  }
  else
    w_desired = 9;
  buggy.w_desired = w_desired;
  delay(dt_millis);
  
}

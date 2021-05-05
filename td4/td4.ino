//used for testing libraries
#include "Arduino.h"
#include "buggy.h"
#include "Ticker.h"

const int dt_millis = 50;
float dt = dt_millis/1000.0;
float sonarDist;
float time_still=0; //move to library along with watchDistance()
float errorDistance_prev = 0;
bool turnaround = 0;

Ticker updateMotorSpeeds; //could not be called inside class, callback() does not exist as in Mbed
Ticker sendPulses; //same as ^^
Ticker lineFollow;
Ticker watchDist;
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
  buggy.left_motor.stop_(); //sets speed to 0 and clears I & D components
  buggy.right_motor.stop_();
  buggy.left_motor.setAngularSpeed(5);
  buggy.right_motor.setAngularSpeed(-5);
  delay(1650);
  turnaround = 0;
  buggy.left_motor.stop_();
  buggy.right_motor.stop_();
}

void stopRunning()
{
  buggy.left_motor.stop_();
  buggy.right_motor.stop_();
  int tol = 0; //time-on-line i.e. time recorded where flag_lineSensor = 1
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
  buggy.stop_ = 0;
}

void watchDistance()
{
  float kp_dist = 0.001; //tuned for max. angular speed of 8 rads/sec, stopping distance of 10cm. To change speed, P values need changing
  float w_desired;
  if (sonarDist > 0) //to cancel out error where no value is returned
  {
    float errorDistance = sonarDist - 12; //steady state of 15cm (5 cm from sonar to wheel axel line/centre of rotation), 2 cm extra for margin of error
    //Proportional-only ontroller as uncertainty from suddenly changing error values can cause serious issues
    w_desired = ((errorDistance * kp_dist)/dt)/0.05;
    
    if (w_desired>8) //9 is a cautionary speed, set a higher one for 2nd submission
    {
      time_still=0;
      w_desired = 8;
    }
    else if (w_desired<1.5) //trapping slow speeds
    {
      w_desired = 0;
      if (time_still > 250) //after 500ms staying still, buggy turns round, reduces error
        if (millis() > 5000) //stops accidental turnaround at beginning
          turnaround = 1;
      time_still+=dt_millis;
    }
    errorDistance_prev = errorDistance;
  }
  else
    w_desired = 9;
  buggy.w_desired = w_desired;
}

void setup()
{
  Serial.begin(9600);
  buggy.servo.write(90);
}


typedef enum {start, follow_line, turn_around, stopBuggy} ProgramState; //program states for finite state machine
ProgramState state = start;

void loop()
{
  switch (state) {
    case (start) :
      updateMotorSpeeds.attach_ms(dt_millis, &adjustSpeeds);
      sendPulses.attach_ms(dt_millis, &getSonarDist);
      lineFollow.attach_ms(dt_millis, &followLine);
      watchDist.attach_ms(dt_millis, &watchDistance);
      state = follow_line;
    break;
    case (follow_line) :
      if (buggy.stop_)
        state = stopBuggy;
      else if (turnaround)
        state = turn_around;
    break;
    case (stopBuggy) :
      updateMotorSpeeds.detach();
      sendPulses.detach();
      lineFollow.detach();
      watchDist.detach();
      stopRunning(); //program will only escape from this function if buggy is replaced on the line
      updateMotorSpeeds.attach_ms(dt_millis, &adjustSpeeds); //returns to original code if put back on line
      sendPulses.attach_ms(dt_millis, &getSonarDist); //only uncommmented so LED didn't annoy me
      lineFollow.attach_ms(dt_millis, &followLine);
      watchDist.attach_ms(dt_millis, &watchDistance);
      state = follow_line;
    break;
    case (turn_around) :
      sendPulses.detach();
      lineFollow.detach(); //stops following the line whilst the buggy turns around
      watchDist.detach();
      turnAround();
      lineFollow.attach_ms(dt_millis, &followLine);
      watchDist.attach_ms(dt_millis, &watchDistance);
      sendPulses.attach_ms(dt_millis, &getSonarDist);
      time_still = 0;
      state = follow_line;
    break;
    default :
      state = start;
  }
  delay(dt_millis);
  
}

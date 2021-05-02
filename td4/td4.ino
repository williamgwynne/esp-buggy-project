//used for testing libraries
#include "Arduino.h"
#include "buggy.h"
#include "Ticker.h"

const int dt_millis = 100;
int dt = dt_millis/1000.0;
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
  //sendPulses.attach_ms(dt_millis, &getSonarDist); //only uncommmented so LED didn't annoy me
  buggy.right_motor.setAngularSpeed(0); //max speed = ~14.93 rads/s
  buggy.left_motor.setAngularSpeed(0);
  buggy.servo.write(90);
}

void loop()
{
  uint32_t lineSensor[7];
  float lineSensor_bool[7];
  int flag_lineSensor;
  float pose_line;
  float error, lastError, output_p, output_i, output_d, output;
  float wr_set, wl_set;

  float w_desired = 10;
  int pose_line_weight[7] = {-4, -2, -1, 0, 1, 2, 4};
  float kp = 3, ki = 1.5, kd = 0.1, dt = 0.1;

  flag_lineSensor = 0;
  buggy.lineSensor.ReadSensor(); // Read one full set of sensor values

  for (int i = 0; i < 7; i++)
  {
    lineSensor[i] = buggy.lineSensor.GetSensorValues(i);
    lineSensor_bool[i] = -(float)lineSensor[i] / 2500.0;
    //Serial.println(lineSensor[i]);
    if (lineSensor[i]<1700)
      flag_lineSensor = 1;
    //flag_lineSensor = (lineSensor[i] < 1700) ? 1 : flag_lineSensor;
   
  }

  if (flag_lineSensor)
  {
    pose_line = 0;
    for (int i = 0; i < 7; i++)
      pose_line += pose_line_weight[i] * lineSensor_bool[i];
    

    lastError = error;
    error = 0 - pose_line;
    output_p = kp * error;
    output_i += ki * error * dt;
    output_d = kd * (error - lastError) / dt;
    output = output_p + output_i + output_d;

    
    wr_set = w_desired + output;
    wl_set = w_desired - output;
    //Serial.println(wr_set);
    buggy.right_motor.setAngularSpeed(wr_set); //max speed = ~14.93 rads/s
    buggy.left_motor.setAngularSpeed(wl_set);
  }
  else
  {
    wr_set = w_desired; //change this to log previous ratios and go at average velocity, rather than straight ahead
    wl_set = w_desired;
    buggy.right_motor.setAngularSpeed(wr_set); //max speed = ~14.93 rads/s
    buggy.left_motor.setAngularSpeed(wl_set);
  }

  delay(100); //Delay before next loop iteration
}

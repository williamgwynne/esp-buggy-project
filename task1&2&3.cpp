#include "Arduino.h"
#include "MotorDriver.h"
#include "ReflectanceSensor.h"

#include "Arduino.h"
#include "buggy.h"
#include "Ticker.h"

const int dt_millis = 100;
float sonarDist;

Motor right_motor(4, 2, 15, 18, 34, 36, dt_millis);
Motor left_motor(2, 1, 12, 0, 39, 35, dt_millis);
Ticker updateMotorSpeeds; //could not be called inside class, callback() does not exist as in Mbed
Ticker sendPulses;        //same as ^^

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
  uint32_t lineSensor[6];
  float lineSensor_bool[6];
  int flag_lineSensor;
  float pose_line;
  float error, lastError, output_p, output_i, output_d, output;
  float wr_set, wl_set;

  float w_desired = 10;
  int pose_line_weight[6] = {-4, -2, -1, 1, 2, 4};
  float kp = 1, ki = 0, kd = 0, dt = 0.03;

  flag_lineSensor = 0;
  sensor.ReadSensor(); // Read one full set of sensor values

  for (int i = 0; i < 6; i++)
  {
    lineSensor[i] = sensor.GetSensorValues(i);
    lineSensor_bool[i] = -(float)lineSensor[i] / 2500;
    flag_lineSensor = (lineSensor[i] < 1700) ? 1 : flag_lineSensor;
  }

  if (flag_lineSensor)
  {
    pose_line = 0;
    for (int i = 0; i < 6; i++)
      pose_line += pose_line_weight[i] * lineSensor_bool[i];

    lastError = error;
    error = 0 - pose_line;
    output_p = kp * error;
    output_i += ki * error * dt;
    output_d = kd * (error - lastError) / dt;
    output = output_p + output_i + output_d;

    wr_set = w_desired + output;
    wl_set = w_desired - output;
  }
  else
  {
    wr_set = w_desired;
    wl_set = w_desired;
  }

  right_motor.setAngularSpeed(wr_set); //max speed = ~14.93 rads/s
  left_motor.setAngularSpeed(wl_set);

  servo.write(90);

  delay(30); //Delay before next loop iteration
}

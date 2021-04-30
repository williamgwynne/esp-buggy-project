//used for testing libraries
#include "Arduino.h"
#include "buggy.h"
#include "Ticker.h"

const int dt_millis = 100;

Motor right_motor(4, 0, 15, 18, 34, 36, dt_millis);
Motor left_motor(2, 0, 12, 0, 39, 35, dt_millis);
Ticker updateMotorSpeeds; //could not be called inside class, callback() foes not exist as in Mbed

void adjustSpeeds() 
{
  right_motor.adjustSpeed();
  left_motor.adjustSpeed();
}

void setup()
{
  Serial.begin(9600);
  updateMotorSpeeds.attach_ms(dt_millis, &adjustSpeeds);
  right_motor.setAngularSpeed(10); //max speed =~2.37 rads/s
  left_motor.setAngularSpeed(10);
}

void loop()
{
//  for (float i=0; i<2.37; i+=0.01) //sweeping throug angular speeds
//  {
//    right_motor.setAngularSpeed(i);
//    left_motor.setAngularSpeed(-i);
//    delay(100);
//  }
//  for (float i=2.37; i>-2.37; i-=0.01)
//  {
//    right_motor.setAngularSpeed(i);
//    left_motor.setAngularSpeed(-i);
//    delay(100);
//  }
//  for (float i=-2.37; i<0; i+=0.01)
//  {
//    right_motor.setAngularSpeed(i);
//    left_motor.setAngularSpeed(-i);
//    delay(100);
//  }
//  sonar.getDist();
}

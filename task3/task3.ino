#include "Arduino.h"
#include "MotorDriver.h"
#include "ReflectanceSensor.h"

//Right Motor Configuration Variables
int motR_pins[3] = {4, 15, 18}; //Define the Motor Pins
int motR_sign = -1;             //Define the motor rotation sign

//Left Motor configuration variables
int motL_pins[3] = {2, 12, 0};
int motL_sign = -1;

uint8_t SensorCount = 6;                          // Number of refectance sensors
uint8_t SensorPins[6] = {23, 22, 19, 27, 25, 32}; // Sensor pins
uint32_t Timeout = 2500;                          // Sensor reflect timeout (us)

MotorDriver Mr;
MotorDriver Ml;
ReflectanceSensor sensor;

float wr_set, wl_set;
float w_desired = 5;
int pose_line_weight[6] = {-4, -2, -1, 1, 2, 4};

void setup()
{
  // Set up the Motors
  //Setup the Right Motor object
  Mr.SetBaseFreq(5000);                                        //PWM base frequency setup
  Mr.SetSign(motR_sign);                                       //Setup motor sign
  Mr.DriverSetup(motR_pins[0], 0, motR_pins[1], motR_pins[2]); //Setup motor pins and channel
  Mr.MotorWrite(0);                                            //Write 0 velocity to the motor when initialising

  //Setup the Left Motor object
  Ml.SetBaseFreq(5000);
  Ml.SetSign(motL_sign);
  Ml.DriverSetup(motL_pins[0], 1, motL_pins[1], motL_pins[2]);
  Ml.MotorWrite(0);

  sensor.SetSensorPins(SensorPins, SensorCount); // Set the sensor pins
  sensor.SetTimeout(Timeout);                    // Set sensor timeout (us)

  //Begin Serial Communication
  Serial.begin(115200);
}

void loop()
{
  uint32_t lineSensor[6];
  float lineSensor_bool[6];
  int flag_lineSensor;
  float pose_line;
  float wr, wl, ur, ul;

  flag_lineSensor = 0;
  sensor.ReadSensor(); // Read one full set of sensor values

  for (int i = 0; i < 6; i++)
  {
    lineSensor[i] = sensor.GetSensorValues(i);
    lineSensor_bool[i] = -lineSensor[i] / 2500.0;
    flag_lineSensor = (lineSensor_bool[i] > 0) ? 1 : flag_lineSensor;
  }

  if (flag_lineSensor)
  {
    pose_line = 0;
    for (int i = 0; i < 6; i++)
      pose_line += pose_line_weight[i] * lineSensor_bool[i];

    float wr_ratio, wl_ratio;

    if (pose_line <0.5) {
      wr_ratio = 1;
      wl_ratio = 0.5;
    } else if (pose_line>0.5) {
      wr_ratio = 0.5;
      wl_ratio = 1;
    } else {
      wr_ratio = 1;
      wl_ratio = 1;
    }
      
    wr_set = w_desired * wr_ratio;
    wl_set = w_desired * wl_ratio;
  }
  else
  {
    wr_set = w_desired;
    wl_set = w_desired;
  }

//  ur = ur + pid_mr.Cal(wr_set, wr);
//  ul = ul + pid_ml.Cal(wl_set, wl);
//
//  ur = (abs(ur) > 1.0) ? (ur / abs(ur)) : ur;
//  ul = (abs(ul) > 1.0) ? (ul / abs(ul)) : ul;
//
  Mr.MotorWrite(wr_set); //Set Velocity percentage to the Motors (-1 to 1)
  Ml.MotorWrite(wl_set);



  delay(30); //Delay before next loop iteration
}

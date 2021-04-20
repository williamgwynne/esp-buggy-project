#include "Arduino.h"
#include "MotorDriver.h"
#include "math.h"
#include "ESP32Encoder.h"

MotorDriver rmotor, lmotor; //DISCONNECT POWER CONNECTOR FROM LINE SENSOR BEFORE UPLOADING
ESP32Encoder EncR, EncL;

double wR_set = 0, wL_set = 0;
double distance = 0;
double w_p_ratio = 901/12500;
double toc_last = 0.0;

//PID coefficients for motor control......................................................................
double errorspeedright_prev = 0, errorspeedleft_prev = 0, errorspeedright_sum = 0, errorspeedleft_sum = 0;
const double kp_speed = 0.3;
const double ki_speed = 0.001;
const double kd_speed = 0.012;
//..........................................................................................................

//PID coefficients for line-speed control...................................................................
const double kp_distance = 144;
const double ki_distance = 0;
const double kd_distance = 1.8;
double errordistance_sum = 0, errordistance_prev = 0;
//..........................................................................................................

void setup()
{

  EncR.attachFullQuad(34, 36);
  //initialising PWM........................................................................................
  rmotor.SetMotorType(DC_MOTOR);            // dc brushed motor
  rmotor.SetBaseFreq(5000);                 // pwm base frequency
  rmotor.DriverSetup(4, 0, 15, 18);         // PwmPin=4, pwm channel=0, PinA=15,PinB=18
  lmotor.SetMotorType(DC_MOTOR);            // dc brushed motor
  lmotor.SetBaseFreq(5000);                 // pwm base frequency
  lmotor.DriverSetup(2, 0, 12, 0);         // PwmPin=2, pwm channel=0, PinA=12,PinB=0
  //........................................................................................................

  Serial.begin(9600); //initiates serial port for communication
  
}

void loop()
{
  double toc = micros()/1000000.0;
  double dt = toc - toc_last;
  toc_last = toc;
  
  float EncR_Speed = (EncR.getCount()/1632.67)/dt; //1632.67 counts per revolution according to manufacturer
  float EncL_Speed = (EncR.getCount()/1632.67)/dt; //1632.67 counts per revolution according to manufacturer
  EncR.clearCount();
  EncL.clearCount(); //force this into a class, EncL_Speed should be a member access function instead, e.g. use EncL.getSpeed()

  
  Serial.println(EncR_Speed, 5);
//  double forwardSpeed = (EncR.Get_Speed()+EncL.Get_speed())/2;
  
  if (distance < 3) {     //stops running after 3 metres, to imitate TD1 task 5
    
  } else {
    wR_set = 0;
    wL_set = 0;
  }
  lmotor.MotorWrite(wL_set);
  rmotor.MotorWrite(wR_set);
}

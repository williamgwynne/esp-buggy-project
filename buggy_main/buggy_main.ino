#include "Arduino.h"
#include "MotorDriver.h"
#include "math.h"
#include "Ticker.h"
#include "ESP32Encoder.h"

MotorDriver rmotor, lmotor; //DISCONNECT POWER CONNECTOR FROM LINE SENSOR BEFORE UPLOADING
ESP32Encoder EncR;
ESP32Encoder EncL;

double wR_set = 1, wL_set = 0.1;
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



//class Encoder : public ESP32Encoder
//{
//  private:
//    float angularSpeed;
//    Ticker speedSampler;
//    void sampleSpeed() {
//      angularSpeed = (getCount()/1632.67)/100; //sampling interval of 100us, 1632.67 counts per revolution (according to manufacturer)
//    }
//  public:
//    Encoder(int encA, int encB) { //constructor
//      attachFullQuad(encA, encB);
//      //speedSampler(callback(this, &Encoder::sampleSpeed), 100) //sampling at 100us
//      enable();
//    }
//    float getAngularSpeed() {
//      return angularSpeed;
//    }
//    void enable() {
//      
//    }
//    void disable() {
//      angularSpeed = 0;
//    }
//  
//};

void setup()
{

  EncR.attachFullQuad(34, 36);
  EncL.attachFullQuad(39, 35);
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
  float EncL_Speed = (EncL.getCount()/1632.67)/dt; //1632.67 counts per revolution according to manufacturer
  EncR.clearCount();
  EncL.clearCount(); //force this into a class, EncL_Speed should be a member access function instead, e.g. use EncL.getSpeed()

//
//  float linearVelocity_left = EncL_Speed *0.05; //v=wr
//  float linearVelocity_right = EncR_Speed *0.05; //v=wr
//  float averageVelocity = (linearVelocity_left + linearVelocity_right)/2.0;
  
  //Right wheel controller...................................................................................
  float errorspeedright = wR_set - EncR_Speed;
  errorspeedright_sum += errorspeedright;
  float uR = (wR_set + (errorspeedright * kp_speed) + (ki_speed * errorspeedright_sum * dt) + ((kd_speed*(errorspeedright-errorspeedright_prev))/dt))/w_p_ratio;
  errorspeedright_prev = errorspeedright;
  //.........................................................................................................

  //Left wheel controller...................................................................................
  float errorspeedleft = wL_set - EncL_Speed;
  errorspeedleft_sum += errorspeedleft;
  float uL = (wL_set + (errorspeedleft * kp_speed) + (ki_speed * errorspeedleft_sum * dt) + ((kd_speed*(errorspeedleft-errorspeedleft_prev))/dt))/w_p_ratio;
  errorspeedleft_prev = errorspeedleft;
  //.........................................................................................................
  
  //Serial.println(uR, 6);

  delay(5); //obviously change this to work with a ticker

  lmotor.MotorWrite(-uL);
  rmotor.MotorWrite(-uR);
}

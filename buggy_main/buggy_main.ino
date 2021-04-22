#include "Arduino.h"
#include "MotorDriver.h"
#include "Ticker.h"
#include "ESP32Encoder.h"
// Callback function header
void callback(char* topic, byte* payload, unsigned int length);

MotorDriver rmotor, lmotor; //DISCONNECT POWER CONNECTOR FROM LINE SENSOR BEFORE UPLOADING
ESP32Encoder EncR;
ESP32Encoder EncL;

float wR_set = 0, wL_set = 0;
float w_p_ratio = 901/12500;
float toc_last = 0.0;
int EncR_lastCount = 0, EncL_lastCount = 0;

//PID coefficients for motor control......................................................................
float dt_millis = 1;
float dt = dt_millis/1000.0;
float errorspeedright_prev = 0, errorspeedleft_prev = 0, errorspeedright_sum = 0, errorspeedleft_sum = 0;
const float kp_speed = 0.3;
const float ki_speed = 0.001;
const float kd_speed = 0.012;
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
//      speedSampler.attach_ms(1, this->sampleSpeed); //sampling at 1ms
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

void setSpeeds() 
{
  
  //Right wheel controller...................................................................................
  float EncR_Speed = ((EncR.getCount()-EncR_lastCount)/1632.67)/dt; //1632.67 counts per revolution according to manufacturer
  EncR_lastCount = EncR.getCount();
  float errorspeedright = wR_set - EncR_Speed;
  errorspeedright_sum += errorspeedright;
  float uR = (wR_set + (errorspeedright * kp_speed) + (ki_speed * errorspeedright_sum * dt) + ((kd_speed*(errorspeedright-errorspeedright_prev))/dt));
  errorspeedright_prev = errorspeedright;
  //EncR.clearCount();
  //.........................................................................................................

  //Left wheel controller...................................................................................
  float EncL_Speed = ((EncL.getCount()-EncR_lastCount)/1632.67)/dt; //1632.67 counts per revolution according to manufacturer
  EncL_lastCount = EncL.getCount();
  float errorspeedleft = wL_set - EncL_Speed;
  errorspeedleft_sum += errorspeedleft;
  float uL = (wL_set + (errorspeedleft * kp_speed) + (ki_speed * errorspeedleft_sum * dt) + ((kd_speed*(errorspeedleft-errorspeedleft_prev))/dt));
  errorspeedleft_prev = errorspeedleft;
  //EncL.clearCount();
  //.........................................................................................................

  rmotor.MotorWrite(-uR);
  lmotor.MotorWrite(-uL);
}

Ticker innerloop;

void setup()
{
  //initialising motors and encoders........................................................................
  EncR.attachFullQuad(34, 36);
  EncL.attachFullQuad(39, 35);
  rmotor.SetMotorType(DC_MOTOR);            // dc brushed motor
  rmotor.SetBaseFreq(5000);                 // pwm base frequency
  rmotor.DriverSetup(4, 0, 15, 18);         // PwmPin=4, pwm channel=0, PinA=15,PinB=18
  lmotor.SetMotorType(DC_MOTOR);            // dc brushed motor
  lmotor.SetBaseFreq(5000);                 // pwm base frequency
  lmotor.DriverSetup(2, 0, 12, 0);         // PwmPin=2, pwm channel=0, PinA=12,PinB=0
  //........................................................................................................

  
  innerloop.attach_ms(1, &setSpeeds);
  
  Serial.begin(9600); //initiates serial port for communication
  
}

void loop()
{

  
}

#include "Arduino.h"
#include "MotorDriver.h"
#include "Ticker.h"
#include "ESP32Encoder.h"
#include "math.h"

// Callback function header
void callback(char* topic, byte* payload, unsigned int length);

class Sonar
{
private:
	float distance;
	long duration;
	int signalPin;
public:
	Sonar(int sigpin) : signalPin(sigpin) {}
	float getDist() 
	{
		
	}
};

class Servo_
{
private:
public:
  Servo_(int pin)
  {
    //pinMode(pin, OUTPUT);
    
  }
  void setAngle(int angle)
  {
    
  }
};

class Motor : private MotorDriver, private ESP32Encoder //includes code for both motor and encoder
{
private:
const float kp = 0.3;
const float ki = 0.001;
const float kd = 0.012;
float w_set;
int encoderA, encoderB;
ESP32Encoder encoder;
float dt;
float encoder_lastCount, errorSpeed_sum, errorSpeed_prev;
public:
  Motor (int pwmPin, int pwmChannel, int pinA, int pinB, int encA, int encB, int dt_ms) : encoderA(encA), encoderB(encB), dt(dt_ms/1000.0)
  //dt_ms is the delay of the ticker
  {
    SetMotorType(DC_MOTOR);
    SetBaseFreq(5000);
    DriverSetup(pwmPin, pwmChannel, pinA, pinB);
    attachFullQuad(encoderA, encoderB);
    encoder_lastCount = 0;
    errorSpeed_sum = 0;
    errorSpeed_prev = 0;
    w_set = 0;
  }
  void adjustSpeed() //ISR, if possible attach to a ticker within this class and move to private
  {
    float encoder_speed = (((getCount()-encoder_lastCount)/1632.67)*2*M_PI)/dt; //1632.67 counts per revolution according to manufacturer
    float errorSpeed = w_set - encoder_speed;
    errorSpeed_sum += errorSpeed;
    float u = (w_set + (errorSpeed * kp) + (ki * errorSpeed_sum * dt) + ((kd*(errorSpeed-errorSpeed_prev))/dt));
    errorSpeed_prev = errorSpeed;
    encoder_lastCount = getCount();
    MotorWrite(-u/14.93); //max angular speed = ~14.93 rads/s
  }
  void setAngularSpeed(float angularSpeed)
  {
    w_set = angularSpeed; //max angular speed = ~14.93 rads/s
  }
  void setLinearSpeed(float linearSpeed)
  {
    //max linear speed = 0.7456m/s
    w_set = linearSpeed/0.05; //w=v/r
  }
};

class Buggy
{
private:
public:
};

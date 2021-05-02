#include "Arduino.h"
#include "MotorDriver.h"
#include "Ticker.h"
#include "ESP32Encoder.h"
#include "math.h"
#include "analogWrite.h"

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
    pinMode (signalPin, OUTPUT);
    digitalWrite(signalPin, HIGH); //start pulse
    delayMicroseconds(10); //perhaps implement ticker in some way
    digitalWrite(signalPin, LOW);
    pinMode (signalPin, INPUT);
    delayMicroseconds(10);
    duration = pulseIn(signalPin, HIGH, 18000); //wait for pulse, timeout after 18000us (stops the program waiting for too long, can be adjusted for shorter equivalet distance)
    distance = duration*0.034/2;
    //Serial.print("Distance: ");
    //Serial.println(distance);
		return distance;
	}
};

class Servo_ //: private Servo
{
private:
int servoPin;
public:
  Servo_(int pin) : servoPin(pin)
  {
    //pinMode(pin, OUTPUT);
//    attach(pin);
  }
  void setAngle(int servoAngle)
  {
    //analogWrite(servoPin, servoAngle);
//    write(servoAngle);
  }
};

class Motor : private MotorDriver, private ESP32Encoder //includes code for both motor and encoder
{
private:
const float kp = 0.3; //PID coefficients are wrong for slow speeds
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
    //Serial.println(encoder_speed);
    float errorSpeed = w_set - encoder_speed;
    errorSpeed_sum += errorSpeed;
    float u = (w_set + (errorSpeed * kp) + (ki * errorSpeed_sum * dt) + ((kd*(errorSpeed-errorSpeed_prev))/dt));
    errorSpeed_prev = errorSpeed;
    encoder_lastCount = getCount();
    MotorWrite(-u/14.93); //max angular speed = ~14.93 rads/s, converts to PWM output -1 .. +1
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

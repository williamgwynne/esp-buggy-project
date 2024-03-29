#include "Arduino.h"
#include "MotorDriver.h"
#include "ReflectanceSensor.h"
#include "Ticker.h"
#include "ESP32Encoder.h"
#include "analogWrite.h"
#include "ESP32Servo.h"

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
		return distance;
	}
};

class Motor : private MotorDriver, private ESP32Encoder //includes code for both motor and encoder
{
private:
const float kp = 1.02;
const float ki = 13.6;
const float kd = 0.051;
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
    setAngularSpeed(linearSpeed/0.05); //w=v/r
  }
  void stop_()
  {
    MotorWrite(0);
  }
};

class Buggy
{
private:
  uint8_t SensorPins[7] = {21, 23, 22, 19, 27, 25, 32}; 
  const float kp = 3;
  const float ki = 1.5;
  const float kd = 0.1;
  float time_off_line;
public:
  float lineError_sum, lineError_prev;
  float w_desired;
  Servo servo; //pwm channel=0
  int dt_millis;
  bool stop_;
  Motor left_motor;
  Motor right_motor;
  Sonar sonar;
  ReflectanceSensor lineSensor;

  Buggy(int dt_millis) : //Pins are set up in this constructor, change pins to corresponding ones on buggy if need be
    dt_millis(dt_millis),
    sonar(14), //signal pin 14
    left_motor(2, 2, 12, 0, 39, 35, dt_millis), //PWM pin=2, PWM channel=1, PinA=12, PinB=0, EncA=39, EncB=35
    right_motor(4, 3, 15, 18, 34, 36, dt_millis), //PWM pin=4, PWM channel=2, PinA=15, PinB=18, EncA=34, EncB=36
    lineSensor() //use the pins as per the TD4 setup, perhaps set up a different class to grab line error and such
  {
    servo.attach(5); //signal pin 5
    lineSensor.SetTimeout(2500);
    lineSensor.SetSensorPins(SensorPins, 7);  
    lineError_sum = 0;
    lineError_prev = 0;
    w_desired = 10;
    stop_=0;
    time_off_line = 0;
  }

  void followLine()
  {
    uint32_t sensorVals[7];
    float lineSensor_bool[7];
    float dt = dt_millis/1000.0;
    int flag_lineSensor;
    float pose_line;
    float error, output_p, output_i, output_d, output;
    float wr_set, wl_set;
    int pose_line_weight[7] = {-4, -2, -1, 0, 1, 2, 4};
    
    
    if(w_desired != 0) {
      flag_lineSensor = 0;
      lineSensor.ReadSensor(); // Read one full set of sensor values
    
      for (int i = 0; i < 7; i++)
      {
        sensorVals[i] = lineSensor.GetSensorValues(i);
        lineSensor_bool[i] = -(float)sensorVals[i] / 2500.0;
        Serial.println(sensorVals[i]);
        if (sensorVals[i]<1250)
          flag_lineSensor = 1;   
      }
    
      if (flag_lineSensor)
      {
        pose_line = 0;
        for (int i = 0; i < 7; i++)
          pose_line += pose_line_weight[i] * lineSensor_bool[i];
        
        time_off_line = 0;
        
        error = 0 - pose_line;
        output_p = kp * error;
        output_i += ki * lineError_sum * dt;
        output_d = kd * (error - lineError_prev) / dt;
        output = output_p + output_i + output_d;
        lineError_prev = error;
  
        //Serial.println(error);
  
         
        wr_set = w_desired + output;//w_desired;
        wl_set = w_desired - output;//w_desired;
        //Serial.println(wr_set);
        right_motor.setAngularSpeed(wr_set); //max speed = ~14.93 rads/s
        left_motor.setAngularSpeed(wl_set);
      }
      else
      {
        wr_set = w_desired; //change this to log previous ratios and go at average velocity, rather than straight ahead
        wl_set = w_desired;
        right_motor.setAngularSpeed(wr_set); //max speed = ~14.93 rads/s
        left_motor.setAngularSpeed(wl_set);
        if(time_off_line>500) //if buggy has been off the track for more than 1/2 second, stop the simulation
          stop_=1;
        time_off_line+=dt_millis;
      }
    } else {
      right_motor.setAngularSpeed(0); //max speed = ~14.93 rads/s
      left_motor.setAngularSpeed(0);
      
    }
  }

  void adjustSpeeds()
  {
    left_motor.adjustSpeed();
    right_motor.adjustSpeed();
  }
};

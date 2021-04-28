#include "Arduino.h"
#include "MotorDriver.h"
#include "ReflectanceSensor.h"
#include "Ticker.h"
#include "ESP32Encoder.h"
// Callback function header
void callback(char* topic, byte* payload, unsigned int length);

uint8_t SensorCount = 6;                          // Number of refectance sensors
uint8_t SensorPins[6] = {23, 22, 19, 27, 25, 32}; // Sensor pins
uint32_t Timeout = 2500;                          // Sensor reflect timeout (us)

float w_desired = 1;
float wR_set = w_desired, wL_set = w_desired;
int pose_line_weight[6] = {-4, -2, -1, 1, 2, 4};
float w_p_ratio = 901/12500;
float toc_last = 0.0;
int EncR_lastCount = 0, EncL_lastCount = 0;

//PID coefficients for motor control......................................................................
float dt_millis = 50;
float dt = dt_millis/1000.0;
float errorspeedright_prev = 0, errorspeedleft_prev = 0, errorspeedright_sum = 0, errorspeedleft_sum = 0;
const float kp_speed = 0.3;
const float ki_speed = 0.001;
const float kd_speed = 0.012;
//..........................................................................................................

MotorDriver rmotor, lmotor; //DISCONNECT POWER CONNECTOR FROM LINE SENSOR BEFORE UPLOADING
ESP32Encoder EncR;
ESP32Encoder EncL;
ReflectanceSensor sensor;



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
  float EncL_Speed = ((EncL.getCount()-EncL_lastCount)/1632.67)/dt; //1632.67 counts per revolution according to manufacturer
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

Ticker innerloop, outerloop;

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


  delay(30); //Delay before next loop iteration
}

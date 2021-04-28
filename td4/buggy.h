#include "Arduino.h"
#include "MotorDriver.h"
#include "Ticker.h"
#include "ESP32Encoder.h"

class Sonar
{
private:
	float distance;
	long duration;
	int signalPin;
public:
	Sonar(int sigpin) : signalPin(sigpin) {}
	float getDist() {
		
	}
};

class Servo
{
private:
public:
};

class Encoder
{
private:
public:
};

class Buggy
{
private:
public:
};
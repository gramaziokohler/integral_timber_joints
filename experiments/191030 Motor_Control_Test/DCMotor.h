// DCMotor.h

#ifndef _DCMOTOR_h
#define _DCMOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class DCMotor
{
    public:
    DCMotor(int pin_ena, int pin_in1, int pin_in2);
    void setSpeed(int speed);
    void setPWM_Deadband(int PWMDeadband);
    void setSpeedPercent(double speed);
	void stop();
    private:
    int _pin_ena;
    int _pin_in1;
    int _pin_in2;
    int _PWMDeadband = 0;
    boolean _running = false;


};

//extern DCMotor DCMotor;

#endif


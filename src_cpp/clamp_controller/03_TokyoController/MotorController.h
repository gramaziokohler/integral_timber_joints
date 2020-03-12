#pragma once
// MotorController.h

#ifndef _MotorController_h
#define _MotorController_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

 // To obtain header definitions
#include <Encoder.h>
#include <DCMotor.h>
#include <MotionProfile.h>
# include <PID_v1.h>

class MotorController {
    public:
    MotorController(DCMotor* motor, Encoder* encoder, double kp, double ki, double kd, double accelStepsPerSecSq) {
        _motor = motor;
        _encoder = encoder;
        _pid = new PID(&_current_position_step, &_motorSpeedPercentage, &_current_target_position_step, kp, ki, kd, DIRECT);
        _pid->SetOutputLimits(-1.0, 1.0);
        _pid->SetSampleTime(_pid_sample_time);
        _accelStepsPerSecSq = accelStepsPerSecSq;
    }

    // Sets the Maximum Power to be applied to DC motor. (maxValue from 0.0 to 1.0)
    void setMaxPower(double maxValue) {
        _pid->SetOutputLimits(-1.0 * abs(maxValue), abs(maxValue));
    }

    boolean moveToPosition(double target_position_step, double velocity) {
        //Sainity check to actually do move
        _movement_target_position_step = target_position_step;
        if (_current_position_step == _movement_target_position_step) return false;

        //Keep track of moving direction
        if (_movement_target_position_step > _current_position_step) {
            _currentDirection = EXTEND;
        } else {
            _currentDirection = RETRACT;
        }

        // Create a new Trapezodial profile and store it
        delete _profile;
        _profile = new TrapezoidalMotionProfile(_current_position_step, target_position_step, velocity, _accelStepsPerSecSq);

        //Start Motion Profile and Run motor
        startProfile();
        return run();
    }

    // Run returns True if a PID computation is performed
    boolean run() {
        // Quite routine if the previous target is reached
        if (!_running_motor) {
            return false;
        }

        // Read encoder
        _current_position_step = _encoder->read();

        // Read motion profile
        _current_target_position_step = _profile->getCurrentStep();

        // Compute PID
        _pid->Compute();

        // Set Motor PWM based on PID Output
        _motor->setSpeedPercent(_motorSpeedPercentage);

        // Stopping condition when target is reached
        // TODO: Unimplemented

        // Stopping condition when error is larger than threshold
        _current_error = _current_position_step - _current_target_position_step;
        if (_current_error < -errorToStop || _current_error > errorToStop) {
            stop();
            return false;
        }
    }

    // Can be called by emergency stop command.
    void stop() {
        _running_motor = false;
        _motor->stop(); // Cut Motor Power In case PID did not reach zero
        _pid->SetMode(0); //Inform PID it had been stopped.
    }

    private:

    void startProfile() {
        // Start the motion profile
        _profile->start();
        //Inform PID to start chasing profile
        _running_motor = true;
        _pid->SetMode(1);
    }

    MotionProfile* _profile;
    DCMotor* _motor;
    Encoder* _encoder;
    PID* _pid;

    double _accelStepsPerSecSq = 1000;
    const double errorToStop = 20;   //TODO: Needs fine tune
    const int _pid_sample_time = 2;  //TODO: Needs fine tune
    boolean _target_reached = true;  //keep track of whether the previous motion profile target is reached
    boolean _running_motor = false;  //keep track if the motor is on. Motor maybe on even after target is reached.

    double _current_position_step = 0;  // Current position                 // Updated by run() from _encoder
    double _current_target_position_step = 0;   // Instantous positional target     // Updated by run() from _profile
    double _movement_target_position_step = 0;   // The goal position set by previous moveTo command.
    double _motorSpeedPercentage = 0.0; // Instantous power ouput for motor // Updated by run() from _pid
    double _current_error = 0;          // Instantous positional error      // Updated by run()
    Direction _currentDirection;

};

enum Direction {
    EXTEND,
    RETRACT
};

#endif

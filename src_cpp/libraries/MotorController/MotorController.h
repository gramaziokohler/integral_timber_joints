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
#include <PID_v1.h>


enum Direction {
    EXTEND,
    RETRACT
};

class MotorController {
    public:
    MotorController(DCMotor* motor, Encoder* encoder, double kp, double ki, double kd, double accelStepsPerSecSq, int run_interval_millis, double errorToStop = 50, boolean encoder_direction = true, boolean controller_direction = true) :
        _accelStepsPerSecSq(accelStepsPerSecSq),
        _controller_run_interval_millis(run_interval_millis),
        _errorToStop(errorToStop),
        _encoder_direction(encoder_direction),
        _controller_direction(controller_direction),
        _motor(motor),
        _encoder(encoder)
    {
        // Create PID controller
        _pid = new PID(&_current_position_step, &_motorSpeedPercentage, &_current_target_position_step, kp, ki, kd, DIRECT);
        _pid->SetSampleTime(_controller_run_interval_millis);
        setMaxPower(1.0);
    }

    // Sets the Maximum Power to be applied to DC motor. (maxValue from 0.0 to 1.0)
    void setMaxPower(const double maxValue) {
        _pid->SetOutputLimits(-1.0 * abs(maxValue), abs(maxValue));
    }

    // Sets the velocity for movements that uses the moveToPosition(const double target_position_step) witout velocity
    void setDefaultVelocity(const double defaultVelocityStepsPerSec) {
        _defaultVelocityStepsPerSec = defaultVelocityStepsPerSec;
    }
    // Sets the paramter for homing. Optional if axis is not home-able.
    void setHomingParam(const uint8_t homingSwitchPin, const uint8_t homingSwitchTriggeredState, const double home_position_step) {
        _homingSwitchPin = homingSwitchPin;
        pinMode(homingSwitchPin, INPUT_PULLUP);
        _homingSwitchTriggeredState = homingSwitchTriggeredState;
        _home_position_step = home_position_step;
    }

    // Create a new motion profile according to target position. (Using default velocity)
    boolean moveToPosition(const double target_position_step) {
        return moveToPosition(target_position_step, _defaultVelocityStepsPerSec);
    }

    // Create a new motion profile according to target position.
    boolean moveToPosition(const double target_position_step, const double velocity) {
        //Sainity check to actually do move
        _current_position_step = getEncoderPos();

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
        return true;
    }

    // Create a new motion profile for homing.
    boolean home(const boolean homeDirectionToNegative, const double homingVelocity) {
        // Sainity check to make sure homing pin is set.
        if (_homingSwitchPin == 0) return false;

        // If the homing switch is pressed (HIGH). Move it until it is not pressed.
        // TODO

        // Keep track of moving direction
        if (homeDirectionToNegative) {
            _currentDirection = RETRACT;
        } else {
            _currentDirection = EXTEND;
        }

        // Target Position
        if (homeDirectionToNegative) {
            _movement_target_position_step = _current_position_step - 100000.0;
        } else {
            _movement_target_position_step = _current_position_step + 100000.0;
        }

        // Create a new Motion profile and store it
        delete _profile;
        _profile = new LinearMotionProfile(_current_position_step, _movement_target_position_step, homingVelocity);

        //Start Motion Profile and Run motor
        _homing = true;
        _homed = false;
        startProfile();
        return true;
        //return run();
    }

    // Run returns True if computation is performed
    boolean run() {
        // Quite routine if the previous target is reached
        if (!_running_motor) {
            return false;
        }

        // Check run interval & time //
        static unsigned long next_run_time = 0;
        if (! millis() > next_run_time) {
            return false;
        }
        next_run_time = millis() + _controller_run_interval_millis;
        // -- -- Check run interval & time -- --

        // Read encoder
        _current_position_step = getEncoderPos();
        // XOR logic below (_encoder_direction != _controller_direction)
        //if (_encoder_direction != _controller_direction) _current_position_step = -_current_position_step;      //Reversed
        
        // Read motion profile
        _current_target_position_step = _profile->getCurrentStep();

        // Compute PID
        if (_pid->Compute()) {

            // Set Motor PWM based on PID Output
            if (_controller_direction) _motor->setSpeedPercent(_motorSpeedPercentage);  //Normal
            else _motor->setSpeedPercent(-_motorSpeedPercentage);                       //Reversed

            // Check stopping conditions
            if (stoppingConditionsMet()) {
                stop();
                return true;
            }
        }
        return true;
    }

    // Can be called by emergency stop command.
    void stop() {
        _running_motor = false;
        _homing = false;
        _motorSpeedPercentage = 0;  // Not sure if I have to do this to reset the value of the PID
        _motor->stop();             // Cut Motor Power In case PID did not reach zero
        _pid->SetMode(0);           // Inform PID it had been stopped.
    }

    boolean isTargetReached() {
        return _target_reached;
    }

    boolean isMotorRunning() {
        return _running_motor;
    }

    boolean isHomed() {
        return _homed;
    }

    boolean isDirectionExtend() {
        return _currentDirection == EXTEND;
    }

    double currentPosition() {
        return _current_position_step;
    }

    double currentTarget() {
        return _current_target_position_step;
    }

    double currentMotorPowerPercentage(){
        return _motorSpeedPercentage;
    }

    // // // Private functions and variables

    private:

    // Start the profile stored in _profile. Setting internal flags.
    void startProfile() {
        // Start the motion profile
        _profile->start();
        //Inform PID to start chasing profile
        _running_motor = true;
        _target_reached = false;
        _pid->SetMode(1);
    }

    int32_t getEncoderPos() {
        // XOR logic below (_encoder_direction != _controller_direction)
        if (_encoder_direction != _controller_direction) return -_encoder->read();
        else return _encoder->read();
    }

    void setEncoderPos(int32_t position) {
        // XOR logic below (_encoder_direction != _controller_direction)
        if (_encoder_direction != _controller_direction) _encoder->write(-position);
        else  _encoder->write(position);
    }

    // Routine to check if a stopping conditions is met. Called by run()
    boolean stoppingConditionsMet() {

        // Stopping condition when target is reached
        if (_currentDirection == EXTEND && _current_position_step >= _movement_target_position_step) {
            _target_reached = true;
            return true; //Break out of condition check.
        }
        if (_currentDirection == RETRACT && _current_position_step <= _movement_target_position_step) {
            _target_reached = true;
            return true; //Break out of condition check.
        }

        // Stopping condition when error is larger than threshold
        _current_error = _current_position_step - _current_target_position_step;
        if (_current_error < -_errorToStop || _current_error > _errorToStop) {
            _target_reached = false;
            return true; //Break out of condition check.
        }

        // Extra stopping condition if the current motion is a homing cycle
        if (_homing) {
            // Stop homing if switch is pressed
            if (digitalRead(_homingSwitchPin) == _homingSwitchTriggeredState) {
                setEncoderPos(_home_position_step);
                _current_position_step = _home_position_step;
                _target_reached = true;
                _homed = true;
                return true; //Break out of condition check.
            }
        }
        return false; // Return false if no conditions are met.
    }

    DCMotor * const _motor;
    Encoder * const _encoder;
    MotionProfile* _profile;
    PID* _pid;

    // Settiing Variables
    const double _errorToStop = 50;                  //TODO: Needs fine tune
    const double _accelStepsPerSecSq = 1000;
    double _defaultVelocityStepsPerSec = 500;
    const int _controller_run_interval_millis = 5;  //TODO: Needs fine tune
    const boolean _encoder_direction = true;        // In the case where Encoder direction do not agree with motor direction.
    const boolean _controller_direction = true;     // In the case where the axis direction needs to be reversed.
    uint8_t _homingSwitchPin = 0;
    uint8_t _homingSwitchTriggeredState = HIGH;
    double _home_position_step = 0;

    // State Variables
    boolean _target_reached = true;     //keep track of whether the previous motion profile target is reached
    boolean _running_motor = false;     //keep track if the motor is on. Motor maybe on even after target is reached.
    boolean _homing = false;            //keep track if the current motion is a homing cycle.
    boolean _homed = false;             //keep track if the system is homed.
    Direction _currentDirection;

    // PID variables
    double _current_position_step = 0;  // Current position                 // Updated by run() from _encoder
    double _current_target_position_step = 0;   // Instantous positional target     // Updated by run() from _profile
    double _movement_target_position_step = 0;   // The goal position set by previous moveTo command.
    double _motorSpeedPercentage = 0.0; // Instantous power ouput for motor // Updated by run() from _pid
    double _current_error = 0;          // Instantous positional error      // Updated by run()
    
};

#endif

/*
 * Generic Stepper Motor Driver Driver
 * Indexer mode only.

 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 *
 * Linear speed profile calculations based on
 * - Generating stepper-motor speed profiles in real time - David Austin, 2004
 * - Atmel AVR446: Linear speed control of stepper motor, 2006
 */
#include "StopStepperDriver.h"

/*
 * Basic connection: only DIR, STEP are connected.
 * Microstepping controls should be hardwired.
 */
StopStepperDriver::StopStepperDriver(short steps, short dir_pin, short step_pin)
:motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin)
{}

StopStepperDriver::StopStepperDriver(short steps, short dir_pin, short step_pin, short enable_pin)
:motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin), enable_pin(enable_pin)
{}
StopStepperDriver::StopStepperDriver(short steps, short dir_pin, short step_pin, short enable_pin, short stop_pinf, short stop_pinr)
  :motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin), enable_pin(enable_pin), stop_pinf(stop_pinf), stop_pinr(stop_pinr)
{}

/*
 * Initialize pins, calculate timings etc
 */
void StopStepperDriver::begin(short rpm, short microsteps){
    pinMode(dir_pin, OUTPUT);
    digitalWrite(dir_pin, HIGH);

    pinMode(step_pin, OUTPUT);
    digitalWrite(step_pin, LOW);

    if IS_CONNECTED(enable_pin){
        pinMode(enable_pin, OUTPUT);
        digitalWrite(enable_pin, HIGH); // disable
    }
    if IS_CONNECTED(stop_pinf){
        pinMode(stop_pinf, INPUT_PULLUP);
    }
    if IS_CONNECTED(stop_pinr){
        pinMode(stop_pinr, INPUT_PULLUP);
    }

    this->rpm = rpm;
    setMicrostep(microsteps);

    enable();
}

/*
 * Set target motor RPM (1-200 is a reasonable range)
 */
void StopStepperDriver::setRPM(short rpm){
    if (this->rpm == 0){        // begin() has not been called (old 1.0 code)
        begin(rpm, microsteps);
    }
    this->rpm = rpm;
}

/*
 * Set stepping mode (1:microsteps)
 * Allowed ranges for StopStepperDriver are 1:1 to 1:128
 */
short StopStepperDriver::setMicrostep(short microsteps){
    for (short ms=1; ms <= getMaxMicrostep(); ms<<=1){
        if (microsteps == ms){
            this->microsteps = microsteps;
            break;
        }
    }
    return this->microsteps;
}

/*
 * Set speed profile - CONSTANT_SPEED, LINEAR_SPEED (accelerated)
 * accel and decel are given in [full steps/s^2]
 */
void StopStepperDriver::setSpeedProfile(Mode mode, short accel, short decel){
    profile.mode = mode;
    profile.accel = accel;
    profile.decel = decel;
}
void StopStepperDriver::setSpeedProfile(struct Profile profile){
    this->profile = profile;
}

/*
 * Move the motor a given number of steps.
 * positive to move forward, negative to reverse
 */
void StopStepperDriver::move(long steps){
    startMove(steps);
    while (nextAction());
}
/*
 * Move the motor a given number of degrees (1-360)
 */
void StopStepperDriver::rotate(long deg){
    move(calcStepsForRotation(deg));
}
/*
 * Move the motor with sub-degree precision.
 * Note that using this function even once will add 1K to your program size
 * due to inclusion of float support.
 */
void StopStepperDriver::rotate(double deg){
    move(calcStepsForRotation(deg));
}

/*
 * Set up a new move or alter an active move (calculate and save the parameters)
 */
void StopStepperDriver::startMove(long steps){
    long speed;
    if (steps_remaining){
        alterMove(steps);
    } else {
        // set up new move
        dir_state = (steps >= 0) ? HIGH : LOW;
        last_action_end = 0;
        steps_remaining = abs(steps);
        step_count = 0;
        rest = 0;
        switch (profile.mode){
        case LINEAR_SPEED:
            // speed is in [steps/s]
            speed = rpm * motor_steps / 60;
            // how many steps from 0 to target rpm
            steps_to_cruise = speed * speed * microsteps / (2 * profile.accel);
            // how many steps are needed from target rpm to a full stop
            steps_to_brake = steps_to_cruise * profile.accel / profile.decel;
            if (steps_remaining < steps_to_cruise + steps_to_brake){
                // cannot reach max speed, will need to brake early
                steps_to_cruise = steps_remaining * profile.decel / (profile.accel + profile.decel);
                steps_to_brake = steps_remaining - steps_to_cruise;
            }
            // Initial pulse (c0) including error correction factor 0.676 [us]
            step_pulse = (1e+6)*0.676*sqrt(2.0f/(profile.accel*microsteps));
            break;
    
        case CONSTANT_SPEED:
        default:
            step_pulse = STEP_PULSE(rpm, motor_steps, microsteps);
            steps_to_cruise = 0;
            steps_to_brake = 0;
        }
    }
}
/*
 * Alter a running move by adding/removing steps
 * FIXME: This is a naive implementation and it only works well in CRUISING state
 */
void StopStepperDriver::alterMove(long steps){
    switch (getCurrentState()){
    case ACCELERATING: // this also works but will keep the original speed target
    case CRUISING:
        if (steps >= 0){
            steps_remaining += steps;
        } else {
            steps_remaining = max(steps_to_brake, steps_remaining+steps);
        };
        break;
    case DECELERATING:
        // would need to start accelerating again -- NOT IMPLEMENTED
        break;
    case STOPPED:
        startMove(steps);
        break;
    }
}
/*
 * Brake early.
 */
void StopStepperDriver::startBrake(void){
    switch (getCurrentState()){
    case CRUISING:  // this applies to both CONSTANT_SPEED and LINEAR_SPEED modes
        steps_remaining = steps_to_brake;
        break;

    case ACCELERATING:
        steps_remaining = step_count * profile.accel / profile.decel;
        break;

    default:
        break; // nothing to do if already stopped or braking
    }
}
/*
 * Stop movement immediately.
 */
void StopStepperDriver::stop(void){
    steps_remaining = 0;
    last_dir_state = dir_state;
    steps_since_last_stop = 0;
}
/*
 * Return calculated time to complete the given move
 */
long StopStepperDriver::getTimeForMove(long steps){
    long t;
    switch (profile.mode){
        case LINEAR_SPEED:
            startMove(steps);
            t = sqrt(2 * steps_to_cruise / profile.accel) + 
                (steps_remaining - steps_to_cruise - steps_to_brake) * STEP_PULSE(rpm, motor_steps, microsteps) +
                sqrt(2 * steps_to_brake / profile.decel);
            break;
        case CONSTANT_SPEED:
        default:
            t = STEP_PULSE(rpm, motor_steps, microsteps);
    }
    return t;
}
/*
 * Move the motor an integer number of degrees (360 = full rotation)
 * This has poor precision for small amounts, since step is usually 1.8deg
 */
void StopStepperDriver::startRotate(long deg){
    startMove(calcStepsForRotation(deg));
}
/*
 * Move the motor with sub-degree precision.
 * Note that calling this function will increase program size substantially
 * due to inclusion of float support.
 */
void StopStepperDriver::startRotate(double deg){
    startMove(calcStepsForRotation(deg));
}

/*
 * calculate the interval til the next pulse
 */
void StopStepperDriver::calcStepPulse(void){
    if (steps_remaining <= 0){  // this should not happen, but avoids strange calculations
        return;
    }

    steps_remaining--;
    step_count++;

    if (profile.mode == LINEAR_SPEED){
        switch (getCurrentState()){
        case ACCELERATING:
            step_pulse = step_pulse - (2*step_pulse+rest)/(4*step_count+1);
            rest = (step_count < steps_to_cruise) ? (2*step_pulse+rest) % (4*step_count+1) : 0;
            break;

        case DECELERATING:
            step_pulse = step_pulse - (2*step_pulse+rest)/(-4*steps_remaining+1);
            rest = (2*step_pulse+rest) % (-4*steps_remaining+1);
            break;

        default:
            break; // no speed changes
        }
    }
}
/*
 * Yield to step control
 * Toggle step and return time until next change is needed (micros)
 */
long StopStepperDriver::nextAction(void){
    if IS_CONNECTED(stop_pinf){ 
	if ((digitalRead(stop_pinf) == LOW) && (dir_state == 1))
	  {
	      Serial.println("stop");
	      stop();
	      return;
	    }
	  }

    if IS_CONNECTED(stop_pinr){ 
	if ((digitalRead(stop_pinr) == LOW) && (dir_state == 0))
	  {
	      Serial.println("stop");
	      stop();
	      return;
	    }
    }
    if (steps_remaining > 0){
        delayMicros(next_action_interval, last_action_end);
        /*
         * DIR pin is sampled on rising STEP edge, so it is set first
         */
        digitalWrite(dir_pin, dir_state);
        digitalWrite(step_pin, HIGH);
        unsigned m = micros();
        long pulse = step_pulse; // save value because calcStepPulse() will overwrite it
        calcStepPulse();
        m = micros() - m;
        // We should pull HIGH for 1-2us (step_high_min)
        if (m < step_high_min){ // fast MCPU or CONSTANT_SPEED
            delayMicros(step_high_min-m);
            m = step_high_min;
        };
        digitalWrite(step_pin, LOW);
        // account for calcStepPulse() execution time; sets ceiling for max rpm on slower MCUs
        last_action_end = micros();
        next_action_interval = (pulse > m) ? pulse - m : 1;
    } else {
        // end of move
        last_action_end = 0;
        next_action_interval = 0;
    }
    return next_action_interval;
}

enum StopStepperDriver::State StopStepperDriver::getCurrentState(void){
    enum State state;
    if (steps_remaining <= 0){
        state = STOPPED;
    } else {
        if (steps_remaining <= steps_to_brake){
            state = DECELERATING;
        } else if (step_count <= steps_to_cruise){
            state = ACCELERATING;
        } else {
            state = CRUISING;
        }
    }
    return state;
}

/*
 * Enable/Disable the motor by setting a digital flag
 */
void StopStepperDriver::enable(void){
    if IS_CONNECTED(enable_pin){
        digitalWrite(enable_pin, LOW);
    }
}

void StopStepperDriver::disable(void){
    if IS_CONNECTED(enable_pin){
        digitalWrite(enable_pin, HIGH);
    }
}

short StopStepperDriver::getMaxMicrostep(){
    return StopStepperDriver::MAX_MICROSTEP;
}

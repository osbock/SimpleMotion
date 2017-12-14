

/*
 * Simple demo, should work with any driver board
 *
 * Connect STEP, DIR as indicated
 *
 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>
#include "StopStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 32

// All the wires needed for full functionality
#define DIR A1
#define STEP A0
//Uncomment line to use enable/disable functionality
#define ENABLE 38

#define XLIMF 3
#define XLIMR 2

// 2-wire basic config, microstepping is hardwired on the driver
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

//Uncomment line to use enable/disable functionality
StopStepperDriver stepper(MOTOR_STEPS, DIR, STEP, ENABLE,XLIMF, XLIMR);

void setup() {
    stepper.begin(RPM, MICROSTEPS);
    Serial.begin(115200);
}

void loop() {

    // energize coils - the motor will hold position
    // stepper.enable();
  
    /*
     * Moving motor one full revolution using the degree notation
     */
   if (Serial.available())
   {
    char c = Serial.read();
    switch (c){
      case 'f':
        Serial.println("forward");
        stepper.enable();
        stepper.move(MOTOR_STEPS*MICROSTEPS*5);
        break;
      case 'r':
        stepper.enable();
        Serial.println("reverse");
        stepper.move(-MOTOR_STEPS*MICROSTEPS*5);
        break;
    }
    
   }
   

    /*
     * Moving motor to original position using steps
     */
    //stepper.move(-MOTOR_STEPS*MICROSTEPS);

    // pause and allow the motor to be moved by hand
    // stepper.disable();


}

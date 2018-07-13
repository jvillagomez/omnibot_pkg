#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Instantiate motorshield objects
Adafruit_MotorShield AFMStop(0x61);
Adafruit_MotorShield AFMSbot(0x60);

// Get motor handler objects from motorshield objects
// documentation on motors states 200 steps per rev

Adafruit_StepperMotor* motorArray[4];

Adafruit_StepperMotor *stepMotor_1 = AFMStop.getStepper(200, 2);
Adafruit_StepperMotor *stepMotor_2 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *stepMotor_3 = AFMSbot.getStepper(200, 2);

float stepMotor1_vel= 20;
float stepMotor2_vel= 20;
float stepMotor3_vel= 20;

void setup()
{  
  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield

  motorArray[1] = stepMotor_1;
  motorArray[2] = stepMotor_2;
  motorArray[3] = stepMotor_3;


  motorArray[1]->setSpeed(stepMotor1_vel); // top 
  motorArray[2]->setSpeed(stepMotor2_vel); // bot
  motorArray[3]->setSpeed(stepMotor3_vel); // bot
}

void loop()
{
  // NOTE => step(#OfSteps, direction, stepType)
  
  // clockwise
  motorArray[1]->step(1, FORWARD, DOUBLE);
  // counter-clockwise
  motorArray[2]->step(1, BACKWARD, DOUBLE);
  // clockwise
  motorArray[3]->step(1, FORWARD, DOUBLE);
}

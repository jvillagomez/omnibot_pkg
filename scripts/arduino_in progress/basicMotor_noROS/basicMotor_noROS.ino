#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Instantiate motorshield objects
Adafruit_MotorShield AFMStop(0x61);
Adafruit_MotorShield AFMSbot(0x60);

// Get motor handler objects from motorshield objects
// documentation on motors states 200 steps per rev
Adafruit_StepperMotor *stepMotor_1 = AFMStop.getStepper(200, 2);
Adafruit_StepperMotor *stepMotor_2 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *stepMotor_3 = AFMSbot.getStepper(200, 2);

// NOTE velocities are in rpm!!!!
float stepMotor1_vel= 60;
float stepMotor2_vel= 60;
float stepMotor3_vel= 60;

void setup()
{  
  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield

  stepMotor_1->setSpeed(stepMotor1_vel); // top 
  stepMotor_2->setSpeed(stepMotor2_vel); // bot
  stepMotor_3->setSpeed(stepMotor3_vel); // bot
}

void loop()
{
  // NOTE => step(#OfSteps, direction, stepType)
  
  // clockwise
  stepMotor_1->step(1, FORWARD, DOUBLE);
  // counter-clockwise
  stepMotor_2->step(1, BACKWARD, DOUBLE);
  // clockwise
  stepMotor_3->step(1, FORWARD, DOUBLE);
}

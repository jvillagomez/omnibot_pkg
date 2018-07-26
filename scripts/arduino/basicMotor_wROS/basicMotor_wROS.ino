#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <omnibot/MotorArray.h>

ros::NodeHandle nh; // Instantiate ros handler object

// Instantiate motorshield objects
// This allows us to then interact with motors attached to our
// motorshields.
Adafruit_MotorShield AFMStop(0x61);
Adafruit_MotorShield AFMSbot(0x60);

// Initialize motor handler objects from the ports on the motorshields.
// AFMStop.getStepper(200, 2) => motorShieldObject.getSTepper
Adafruit_StepperMotor *stepMotor_1 = AFMStop.getStepper(200, 2);
Adafruit_StepperMotor *stepMotor_2 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *stepMotor_3 = AFMSbot.getStepper(200, 2);

// Declare array of step motors
// Array is declared to 4 members, but we only index motors from 1-3
// This was done for clarity of new users.
Adafruit_StepperMotor* motorArray[4];

// Initialize motors at angular_vel=0; available globally
float velocityArray[4] = {0}; 

// Step motors will vibrate regardless of what velocity is sent to the motor.
// If motor velocity is '0', the motor will still vibrate.
// To avoid this, we define a thershold. if the desired velocity is below 
// this threshold, no step will be issued to the motor.
// The motor velocity variables will still reflect this desired velosity, 
// even if it crosses the threshold.
float velocityThreshold = 0.1;

// Declare custom made variables that will hold our velocities and displacements,
// that will be published to their respective topics.
omnibot::MotorArray currentVelocities;
omnibot::MotorArray angularDisplacements;


// callback function for subscriber below. Updates velocities from FIFO queue
void updateMotorVelocities( const omnibot::MotorArray& velocity_msg)
{
  velocityArray[1] = velocity_msg.motor1; velocityArray[2] = velocity_msg.motor2; velocityArray[3] = velocity_msg.motor3;

  motorArray[1]->setSpeed(abs(velocityArray[1]));
  motorArray[2]->setSpeed(abs(velocityArray[2]));
  motorArray[3]->setSpeed(abs(velocityArray[3]));
}

bool velocityIsAboveThreshold(float motorVel, float threshold)
{
  if ( abs(motorVel) > threshold ) {
    return true;
  }
  return false;
}

float getCurrentMotorVelocity(float velocity, float threshold)
{
  
  if (velocityIsAboveThreshold(velocity, threshold)) {
    return velocity;
  }
  return 0;
}

float getAngularDisplacement(float velocity)
{
  if (velocity > 0) {
    return 0.0314159;
  }
  else if(velocity < 0) {
    return -0.0314159;
  }
  else {
    return 0;
  } 
}

void motorStep(float motorVel, int motorNumber)
{
  if (velocityIsAboveThreshold(motorVel, velocityThreshold)) {
    if(motorVel < 0) {
      motorArray[motorNumber]->step(1, BACKWARD, DOUBLE); // clockwise
    }
    else {
      motorArray[motorNumber]->step(1, FORWARD, DOUBLE); // counter-clockwise
    }
  }
}

ros::Publisher currentMotorVelocities_topic("currentMotorVelocities_topic", &currentVelocities);
ros::Publisher angularDisplacements_topic("angularDisplacements_topic", &angularDisplacements);

void publishCurrentMotorVelocities()
{
  currentVelocities.motor1 = getCurrentMotorVelocity(velocityArray[1], velocityThreshold);
  currentVelocities.motor2 = getCurrentMotorVelocity(velocityArray[2], velocityThreshold);
  currentVelocities.motor3 = getCurrentMotorVelocity(velocityArray[3], velocityThreshold);

  currentMotorVelocities_topic.publish( &currentVelocities );
}

void publishAngularDisplacements()
{ 
  angularDisplacements.motor1 = getAngularDisplacement(currentVelocities.motor1);
  angularDisplacements.motor2 = getAngularDisplacement(currentVelocities.motor2);
  angularDisplacements.motor3 = getAngularDisplacement(currentVelocities.motor3);

  angularDisplacements_topic.publish( &angularDisplacements );
}


// Set up motorVelocities subscriber
ros::Subscriber<omnibot::MotorArray> motorVelocities("setStepMotorVelocity_topic", &updateMotorVelocities );

void activateMotors()
{
  for(int i = 1; i < 4; i++)
  {
    motorStep(velocityArray[i], i);
  }
}

void setup()
{
  Serial.begin(115200); // set baud rate
  nh.initNode();

  // notify master of our new publishers and subscribers
  nh.subscribe(motorVelocities);
  nh.advertise(currentMotorVelocities_topic);
  nh.advertise(angularDisplacements_topic);
  
  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield

  // add motor instances to our motorArray
  // ommited index=0 for clarity
  motorArray[1] = stepMotor_1;
  motorArray[2] = stepMotor_2;
  motorArray[3] = stepMotor_3;
}


void loop()
{
  nh.spinOnce();

  // for(int i = 1; i < 4; i++)
  // {
  //   motorStep(velocityArray[i], i);
  // }
  
  activateMotors();
  publishCurrentMotorVelocities();
  publishAngularDisplacements();
}
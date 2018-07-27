#include <Wire.h>
#include <Adafruit_MotorShield.h> // library nbeeded to interact with motorshields
#include <ros.h> // library needed for ROS communication
#include <omnibot/MotorArray.h> // library needed for custom ROS 'MotorArray' msg type

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

// Callback function for 'setStepMotorVelocity_topic' subscriber below. 
// Updates velocity variables from FIFO queue, but does not send a step 
// command to the motors.
// The velocityArray is globally available. After this function updates 
// its values, the array is available for any other function to use.
// The desired values are assign to the velocityArray, rgardless of 
// whether they are wihtin the threshold or not.
void updateMotorVelocities( const omnibot::MotorArray& velocity_msg)
{
  // assign velocityArray values as recieved from 'setStepMotorVelocity_topic' 
  velocityArray[1] = velocity_msg.motor1; velocityArray[2] = velocity_msg.motor2; velocityArray[3] = velocity_msg.motor3;

  // The absolute value of the velocityArray values is sent to the motors.
  // Negative values sent to the motors cause incorrect rotation.
  // To actualize direction (CW and CCW), we use the "BACKWARD"/"FORWARD" parameters in the step command.
  motorArray[1]->setSpeed(abs(velocityArray[1]));
  motorArray[2]->setSpeed(abs(velocityArray[2]));
  motorArray[3]->setSpeed(abs(velocityArray[3]));
}

// Decides whether a desired velocity (absolute value of desired velocity) 
// is above (TRUE) the threshold, or below (FALSE).
bool velocityIsAboveThreshold(float motorVel, float threshold)
{
  if ( abs(motorVel) > threshold ) {
    return true;
  }
  return false;
}

// Grabs the values of the desired velocities, but takes the threshold 
// value nto consideration. If the value is above the threshold, 
// the velocity is returned. if value is below th ethreshold, a '0' is returned.
float getCurrentMotorVelocity(float velocity, float threshold)
{ 
  if (velocityIsAboveThreshold(velocity, threshold)) {
    return velocity;
  }
  return 0;
}

// Calculates the angular displacement, depending on the direction of rotation.
// Our step motors are 200step motors. (Take 200 steps to rotate 360 degrees).
// With each step being 1.8degrees and our velocities being relatively slow, w ecan
// estimate our angular displacement per step as -1.8 || +1.8 dgerees.
// Positive angular velocity (CCW) +1.8 == +0.0314159
// Negative angular velocity (CW) -1.8 == -0.0314159
// If velocity is zero (ie scenarios where vleocity is below threshold), a '0' is
// returned for no angular displacement.
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

// Sends step command to the motors.
// Takes threhsold into consideration. If desired velocity is '0' (ie like when below threshold),
// then no step is issued. No need to send a step command if the velocity is zero.
// This takes positive/negative signs into consideration. If desired velocity is
// Positive angular velocity (CCW) == BACKWARD
// Negative angular velocity (CW) == FORWARD
// Step command is below:
// motorArray[motorNumber]->step(#OfSteps, DIRECTION, stepTYPE)
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

// Initializes the publishers that report our current velocities and 
// angular displacmeents to the Pi.
ros::Publisher currentMotorVelocities_topic("currentMotorVelocities_topic", &currentVelocities);
ros::Publisher angularDisplacements_topic("angularDisplacements_topic", &angularDisplacements);

// Assigns the velocities from our currently desired values to our
// custom motorArray msg object.
void publishCurrentMotorVelocities()
{
  currentVelocities.motor1 = getCurrentMotorVelocity(velocityArray[1], velocityThreshold);
  currentVelocities.motor2 = getCurrentMotorVelocity(velocityArray[2], velocityThreshold);
  currentVelocities.motor3 = getCurrentMotorVelocity(velocityArray[3], velocityThreshold);

  currentMotorVelocities_topic.publish( &currentVelocities );
}

// Assigns the angular displacement (WRT our current iteration's velocity) toour
// custom motorArray msg object.
void publishAngularDisplacements()
{ 
  angularDisplacements.motor1 = getAngularDisplacement(currentVelocities.motor1);
  angularDisplacements.motor2 = getAngularDisplacement(currentVelocities.motor2);
  angularDisplacements.motor3 = getAngularDisplacement(currentVelocities.motor3);

  angularDisplacements_topic.publish( &angularDisplacements );
}

// Set up motorVelocities subscriber
// The velocity for each stepMotor is declared inside of a custom motorArray ROS msg
// and is recioeved through the 'setStepMotorVelocity_topic'.
ros::Subscriber<omnibot::MotorArray> motorVelocities("setStepMotorVelocity_topic", &updateMotorVelocities );

// Iterates through the stepMotor array and performs the step command (if applicable).
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
  nh.initNode(); // Initialize ROSserial node

  // notify master of our new publishers and subscribers
  nh.subscribe(motorVelocities);
  nh.advertise(currentMotorVelocities_topic);
  nh.advertise(angularDisplacements_topic);
  
  AFMSbot.begin(); // Initialize the bottom shield
  AFMStop.begin(); // Initialize the top shield

  // add motor instances to our motorArray
  // ommited index=0 for clarity
  motorArray[1] = stepMotor_1;
  motorArray[2] = stepMotor_2;
  motorArray[3] = stepMotor_3;
}

void loop()
{
  // This causes ROS to cycle.
  // Normally (on a multithreaded device) each subscriber
  // works in its own thread. Because this device is 
  // single threaded, this function aids in protothreading and making
  // sure our values are updated accordingly.
  nh.spinOnce();
  
  // Processes current desired velocities and sends the 
  activateMotors();
  
  // step commands ot the motors// Publishes currrent motor velocities to 'currentMotorVelocities_topic'.
  publishCurrentMotorVelocities();  
  // Publishes the angular displacement for the current iteration to 'angularDisplacements_topic'.
  publishAngularDisplacements();
}
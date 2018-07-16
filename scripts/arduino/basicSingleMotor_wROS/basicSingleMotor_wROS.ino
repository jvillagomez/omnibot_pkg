#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <omnibot/MotorArray.h>
// ==================================================
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh; // Instantiate ros handler object

// Instantiate motorshield objects
Adafruit_MotorShield AFMStop(0x61);
Adafruit_MotorShield AFMSbot(0x60);

// Get motor handler objects from motorshield objects
Adafruit_StepperMotor *stepMotor_1 = AFMStop.getStepper(200, 2);

Adafruit_StepperMotor* motorArray[4];
float velocityArray[4] = {0}; // initialize motors at angular_vel=0; available globally
float previousVelocities[4] = {0};

float velocityThreshold = 0.1;

omnibot::MotorArray currentVelocities;

// callback function for subscriber below. Updates velocities from FIFO queue
void updateMotorVelocities( const omnibot::MotorArray& velocity_msg)
{
  velocityArray[1] = velocity_msg.motor1; velocityArray[2] = velocity_msg.motor2; velocityArray[3] = velocity_msg.motor3;

  motorArray[1]->setSpeed(abs(velocityArray[1]));
}

bool velocityIsAboveThreshold(float motorVel, float threshold)
{
  if ( abs(motorVel) > threshold ) {
    return true;
  }
  return false;
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

void publishCurrentMotorVelocities()
{
  currentVelocities.motor1 = velocityArray[1];
}

// Set up motorVelocities subscriber
ros::Subscriber<omnibot::MotorArray> motorVelocities("setStepMotorVelocity_topic", &updateMotorVelocities );
ros::Publisher currentMotorVelocities_topic("currentMotorVelocities_topic", &currentVelocities);

void setup()
{
  Serial.begin(115200); // set baud rate
  nh.initNode();

  // notify master of our new publishers and subscribers
  nh.subscribe(motorVelocities);
  nh.advertise(currentMotorVelocities_topic);
  
  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield

  // add motor instances to our motorArray
  // ommited index=0 for clarity
  motorArray[1] = stepMotor_1;
}

void loop()
{
  nh.spinOnce();

  motorStep(velocityArray[1], 1);

  publishCurrentMotorVelocities();
  currentMotorVelocities_topic.publish( &currentVelocities );
}
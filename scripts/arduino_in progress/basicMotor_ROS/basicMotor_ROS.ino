#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
// ==================================================
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh; // Instantiate ros handler object

// Instantiate motorshield objects
Adafruit_MotorShield AFMStop(0x61);
Adafruit_MotorShield AFMSbot(0x60);

// Get motor handler objects from motorshield objects
Adafruit_StepperMotor *stepMotor_1 = AFMStop.getStepper(200, 2);
Adafruit_StepperMotor *stepMotor_2 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *stepMotor_3 = AFMSbot.getStepper(200, 2);

Adafruit_StepperMotor* motorArray[4];
float velocityArray[4] = {0}; // initialize motors at angular_vel=0; available globally
float velocityThreshold = 0.1;

geometry_msgs::Point currentVelocities;


// Set up motorVelocities subscriber
ros::Subscriber<geometry_msgs::Point> motorVelocities("setStepMotorVelocity_topic", &updateMotorVelocities );
ros::Publisher currentMotorVelocities_topic("currentMotorVelocities_topic", &currentVelocities);

// // callback function for subscriber below. Updates velocities from FIFO queue
void updateMotorVelocities( const geometry_msgs::Point& velocity_msg)
{
  velocityArray[1] = velocity_msg.x; velocityArray[2] = velocity_msg.y; velocityArray[3] = velocity_msg.z;

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
  currentVelocities.x = velocityArray[1];
  currentVelocities.y = velocityArray[2];
  currentVelocities.z = velocityArray[3];
}

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
  motorArray[2] = stepMotor_2;
  motorArray[3] = stepMotor_3;
}

void loop()
{
  nh.spinOnce();

  for(int i = 1; i < 4; i++)
  {
    motorStep(velocityArray[i], i);
  }

  publishCurrentMotorVelocities();
  currentMotorVelocities_topic.publish( &currentVelocities );
}

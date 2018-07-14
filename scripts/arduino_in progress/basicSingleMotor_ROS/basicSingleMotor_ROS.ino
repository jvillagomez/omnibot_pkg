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

// initialize motors at angular_vel=0; available globally
float motor1_desiredVel = 0;
float velocityThreshold = 0.1;

// // callback function for subscriber below. Updates velocities from FIFO queue
void updateMotorVelocities( const geometry_msgs::Point& velocity_msg)
{
  motor1_desiredVel = velocity_msg.x;
//  stepMotor_1->setSpeed(modifyVelocityForSmootherSteps(motor1_desiredVel));
  stepMotor_1->setSpeed(abs(motor1_desiredVel));
}


// used to turn velocities (RPMs) into negative values.
// TODO investigate why stepper motors respond better to negative values
//float modifyVelocityForSmootherSteps(float desiredVelocity)
//{
//  if (desiredVelocity < 0) {
//    return (-1*desiredVelocity);
//  }
//  return desiredVelocity;
//}

bool velocityIsAboveThreshold(float motorVel, float threshold)
{
  if ( abs(motorVel) > threshold ) {
    return true;
  }
  return false;
}

void motor1Step(float motorVel)
{
  if (velocityIsAboveThreshold(motorVel, velocityThreshold)) {
    if(motorVel < 0) {
      stepMotor_1->step(1, BACKWARD, DOUBLE); // clockwise
    }
    else {
      stepMotor_1->step(1, FORWARD, DOUBLE); // counter-clockwise
    }
  }
}

// Set up motorVelocities subscriber
ros::Subscriber<geometry_msgs::Point> motorVelocities("setStepMotorVelocity_topic", &updateMotorVelocities );
// ros::Publisher currentMotorVelocities_topic("currentMotorVelocities_topic", &currentMotorVelocities);

void setup()
{
  Serial.begin(115200); // set baud rate
  nh.initNode();

  // notify master of our new publishers and subscribers
  nh.subscribe(motorVelocities);
  // nh.advertise(currentMotorVelocities_topic);
  
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
  motor1Step(motor1_desiredVel);
}

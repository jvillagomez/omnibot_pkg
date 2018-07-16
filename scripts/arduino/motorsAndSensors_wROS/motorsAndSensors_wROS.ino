// =================================================
// Motor Libraries [START] -------------------------
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <omnibot/MotorArray.h>
// Motor Libraries [END] ---------------------------
// =================================================
// =================================================
// Sensor Libraries [START] ------------------------
#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <geometry_msgs/Vector3.h>
// Sensor Libraries [END] --------------------------
// =================================================

ros::NodeHandle nh; // Instantiate ros handler object

// ==============================================================
// Motor variable declaration [START]----------------------------
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

omnibot::MotorArray currentVelocities;
omnibot::MotorArray angularDisplacements;

ros::Publisher currentMotorVelocities_topic("currentMotorVelocities_topic", &currentVelocities);
ros::Publisher angularDisplacements_topic("angularDisplacements_topic", &angularDisplacements);
// Motor variable declaration [END]------------------------------
// ==============================================================

// ==============================================================
// Sensor variable declaration [START]----------------------------
// preallocate Vector3 variables to hold orientation/acceleration
geometry_msgs::Vector3 orientation;
geometry_msgs::Vector3 linearAccel;
geometry_msgs::Vector3 angularAccel;

// Set up acceleation and orienation publishers
ros::Publisher linearAccel_topic("linearAccel_topic", &linearAccel);
ros::Publisher angularAccel_topic("angularAccel_topic", &angularAccel);
ros::Publisher orientation_topic("orientation_topic", &orientation);

// instantiate Madgwick filter object
Madgwick filter;

// initialize sensor data variables
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

int aix, aiy, aiz;
int gix, giy, giz;

float ax, ay, az;
float gx, gy, gz;

float roll, pitch, heading;
unsigned long microsNow;
// Sensor variable declaration [END]------------------------------
// ==============================================================

// ==============================================================
// Motor function and callbacks [START]--------------------------
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
// Motor function and callbacks [END]----------------------------
// ==============================================================


// ==============================================================
// Sensor function and callbacks [START]--------------------------
float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void publishLinearAndAngularAccel()
{
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);

  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  linearAccel.x = ax;
  linearAccel.y = ay;
  linearAccel.z = az;
  
  angularAccel.x = gx;
  angularAccel.y = gy;
  angularAccel.z = gz;

  linearAccel_topic.publish( &linearAccel );
  angularAccel_topic.publish( &angularAccel );
}

void publishOrientation()
{
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

  orientation.x = roll;
  orientation.y = pitch;
  orientation.z = heading;

  orientation_topic.publish( &orientation );
}

void processAndPublishSensorData()
{
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    publishLinearAndAngularAccel();
    publishOrientation();

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}
// Sensor function and callbacks [END]----------------------------
// ==============================================================

void setup()
{
  // =============================================
  // Motor setup [START]--------------------------
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
  // Motor setup [END]--------------------------
  // ===========================================

  // =============================================
  // Sensor setup [START]--------------------------
  nh.advertise(orientation_topic);
  nh.advertise(linearAccel_topic);
  nh.advertise(angularAccel_topic);
  
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
  // Sensor setup [END]--------------------------
  // ===========================================
}


void loop()
{
  nh.spinOnce(); // very important to understand for protothreading

  // ====================================================
  // Motor loop [START]----------------------------------
  activateMotors();
  publishCurrentMotorVelocities();
  publishAngularDisplacements();
  // Motor loop [END]----------------------------------
  // ====================================================
  // ====================================================
  // Sensor loop [START]----------------------------------
  processAndPublishSensorData();
  // Sensor loop [END]----------------------------------
  // ====================================================
}
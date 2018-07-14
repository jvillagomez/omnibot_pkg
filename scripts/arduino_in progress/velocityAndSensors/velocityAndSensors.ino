#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
// ==================================================
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh; // Instantiate ros handler object


// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);
//
// char hello[13] = "hello world!";


// Instantiate motorshield objects
Adafruit_MotorShield AFMStop(0x61);
Adafruit_MotorShield AFMSbot(0x60);

// Get motor handler objects from motorshield objects
Adafruit_StepperMotor *stepMotor_1 = AFMStop.getStepper(200, 2);
Adafruit_StepperMotor *stepMotor_2 = AFMSbot.getStepper(200, 1);
Adafruit_StepperMotor *stepMotor_3 = AFMSbot.getStepper(200, 2);

// initialize motors at angular_vel=0; available globally
float stepMotor1_vel= 0;
float stepMotor2_vel= 0;
float stepMotor3_vel= 0;

// SET UP ONBOARD SENSORS [START] --------------------
// ---------------------------------------------------
// preallocate Vector3 variables to hold orientation/acceleation
geometry_msgs::Vector3 orientation;
geometry_msgs::Vector3 linearAccel;
geometry_msgs::Vector3 angularAccel;
// std_msgs::Float32MultiArray currentMotorVelocities;


// // callback function for subscriber below. Updates velocities from FIFO queue
void updateMotorVelocities( const geometry_msgs::Point& velocity_msg)
{
  // digitalWrite(13, HIGH-digitalRead(13));   // blink the led

  stepMotor1_vel = velocity_msg.x;
  stepMotor2_vel = velocity_msg.y;
  stepMotor3_vel = velocity_msg.z;

  stepMotor_1->setSpeed(stepMotor1_vel);
  stepMotor_2->setSpeed(stepMotor2_vel);
  stepMotor_3->setSpeed(stepMotor3_vel);
}

void setMotor1Velocity(float motorVel)
{
  if ( abs(motorVel) > 0.01 ) {
    if(motorVel < 0) {
        stepMotor_1->step(BACKWARD, DOUBLE);
    }
    stepMotor_1->step(FORWARD, DOUBLE);
  }
}
void setMotor2Velocity(float motorVel)
{
  if ( abs(motorVel) > 0.01 ) {
    if(motorVel < 0) {
        stepMotor_2->step(BACKWARD, DOUBLE);
    }
    stepMotor_2->step(FORWARD, DOUBLE);
  }
}
void setMotor3Velocity(float motorVel)
{
  if ( abs(motorVel) > 0.01 ) {
    if(motorVel < 0) {
        stepMotor_3->step(BACKWARD, DOUBLE);
    }
    stepMotor_3->step(FORWARD, DOUBLE);
  }
}

// Set up motorVelocities subscriber
ros::Subscriber<geometry_msgs::Point> motorVelocities("setStepMotorVelocity_topic", &updateMotorVelocities );

// Set up acceleation and orienation publishers
ros::Publisher linearAccel_topic("linearAccel_topic", &linearAccel);
ros::Publisher angularAccel_topic("angularAccel_topic", &angularAccel);
ros::Publisher orientation_topic("orientation_topic", &orientation);
// ros::Publisher currentMotorVelocities_topic("currentMotorVelocities_topic", &currentMotorVelocities);

//
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

// onboard-sensor helper functions
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

// ---------------------------------------------------
// SET UP ONBOARD SENSORS [END] -----------------------


void setup()
{
  Serial.begin(115200); // set baud rate
  nh.initNode();
  // nh.advertise(chatter);

  // // notify master of our new publishers and subscribers
  nh.subscribe(motorVelocities);
  nh.advertise(orientation_topic);
  nh.advertise(linearAccel_topic);
  nh.advertise(angularAccel_topic);
  //
  // nh.advertise(currentMotorVelocities_topic);
  // nh.advertise(motorAngularDisplacements_topic);
  //
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

  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield
}

void loop()
{
  // str_msg.data = hello;
  // chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);

  setMotor1Velocity(stepMotor1_vel);
  setMotor3Velocity(stepMotor2_vel);
  setMotor2Velocity(stepMotor3_vel);
  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    linearAccel.x = ax;
    linearAccel.y = ay;
    linearAccel.z = az;
    angularAccel.x = gx;
    angularAccel.y = gy;
    angularAccel.z = gz;

    orientation.x = roll;
    orientation.y = pitch;
    orientation.z = heading;

    linearAccel_topic.publish( &linearAccel );
    angularAccel_topic.publish( &angularAccel );
    orientation_topic.publish( &orientation );

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
  // currentMotorVelocities.data[0] = stepMotor1_vel;
  // currentMotorVelocities.data[1] = stepMotor2_vel;
  // currentMotorVelocities.data[2] = stepMotor3_vel;
  // currentMotorVelocities_topic.publish(&currentMotorVelocities);
}

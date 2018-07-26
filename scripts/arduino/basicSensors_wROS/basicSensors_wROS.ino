#include <ros.h> // ROS library
#include <CurieIMU.h> // accelerometer library
#include <MadgwickAHRS.h> // noise filter library
#include <geometry_msgs/Vector3.h> // ROS messages library

// Instantiate ROS handler object.
// (this makes it a ROS node)
ros::NodeHandle nh; 

// preallocate Vector3 variables to hold:
// --------------------------------------
// var orientation = orientation 
// var linearAccel = linear acceleration
// var angularVel = angular velocity
geometry_msgs::Vector3 orientation; 
geometry_msgs::Vector3 linearAccel;
geometry_msgs::Vector3 angularVel;

// Set up publishers for each of the variables above.
// Each of these publishers will publish each of the variables above,
// every time the arduino loop executes (activated by the "spinOnce" on line 141).
ros::Publisher orientation_topic("orientation_topic", &orientation);
ros::Publisher linearAccel_topic("linearAccel_topic", &linearAccel);
ros::Publisher angularVel_topic("angularVel_topic", &angularVel);

// Create a Madgwick object to access the functions from the 
// Madgwick class in the library. Here, we call it filter.
// A prewritten class from ADAFRUIT website that will allow 
// us to get a reading from the onboard gyroscope and accelerometer.
// https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser
Madgwick filter;

// initialize sensor data variables
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

// The algorithm takes raw values from a gyroscope and accelerometer, 
// and uses them to return four quaternions:

int aix, aiy, aiz;
int gix, giy, giz;

float ax, ay, az;
float gx, gy, gz;

// which are 4-dimensional numbers which contain x, y, and z values to 
// represent the axis around which rotation occurs, as well as a ω value
// which represents the value of rotation which occurs around the same 
// axis. These quaternions can be used to calculate the Euler angles 
// pitch, yaw, and roll.

float roll, pitch, heading;
unsigned long microsNow;

// Helper functions for translating quarternion readings.
// Both are used in 'publishLinearAccelAndangularVel()'
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

// This function takes care of three different tasks
// 1) Translatemrat raw quarternion readings to:
//    -Gs for linear accelration (line )
//    -Degrees/sec for angular velocity
// 2) Publish tranlated motion values to their respective topics
// 3) Update the the Madgwick object (the filter) to later obtain 
//    our roll, pitch, and yaw
void publishLinearAccelAndangularVel()
{
  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);
  
  // update the filter, which computes orientation
  // we will call this filter in a separata function to get roll, pitch, yaw
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // attach linear accel and rotational vel values for publishing
  linearAccel.x = ax;
  linearAccel.y = ay;
  linearAccel.z = az;
  angularVel.x = gx;
  angularVel.y = gy;
  angularVel.z = gz;

  // publish values
  linearAccel_topic.publish( &linearAccel );
  angularVel_topic.publish( &angularVel );
}

// Retrieves roll, pitch, and yaw from the filter (Madgqiwck object)
// and publishes it to the orientation topic.
// Roll, pitch, and yaw are calculated according to the update performed on the filter 
// from 'publishLinearAccelAndangularVel()' function (runs beforehand).
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

// Checks to ensure its time to take a new reading, according to timestep defined
// inside of the 'setup()' function.
// If its time for a reading, linearAcceleration, angularVelocity,a nd orientation are
// all read and published.
void processAndPublishSensorData()
{
  // Timestep is calculated below.
  // Configuration for the itme step is set in the 'setup()' function.
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU and store in 'aix, aiy, aiz, gix, giy, giz'
    // These variables are available globally!
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    publishLinearAccelAndangularVel();
    publishOrientation();

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

void setup()
{
  // Set baud rate for serial transmission.
  Serial.begin(115200); 
  // Initilizes the ROS node.
  nh.initNode();  

  // Notify the master node that we have 3 publishers declared above.
  nh.advertise(orientation_topic);
  nh.advertise(linearAccel_topic);
  nh.advertise(angularVel_topic);
  
  // Start the IMU and perform preliminary configuration by 
  // setting the sample rate of the acelerometer and the gyro 
  // and the filter to 25Hz:
  CurieIMU.begin();
  CurieIMU.setGyroRate(25); //25 Hz
  CurieIMU.setAccelerometerRate(25); // 25Hz
  filter.begin(25); // 25Hz

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

void loop()
{
  // This causes ROS to cycle.
  // Normally (on a multithreaded device) each subscriber
  // works in its own thread. Because this device is 
  // single threaded, this function aids in protothreading and making
  // sure our values are updated accordingly.
  nh.spinOnce();

  // This line processes all other functions, preparing and publishing 
  // all outgoing data.
  processAndPublishSensorData();
}

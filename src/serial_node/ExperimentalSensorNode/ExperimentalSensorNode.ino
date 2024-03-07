#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "functions.h"
#include "macros.h"

ros::NodeHandle nh;

// Globals
unsigned int event; 

// IMU Variables
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28, &Wire);
adafruit_bno055_offsets_t calibrationData;
sensors_event_t gravityData, orientationData;


// IMU Publishers
std_msgs::Int16 imuBearing;
std_msgs::Int16 imuGrav;
ros::Publisher imubpub("IMU_Bearing", &imuBearing);
ros::Publisher imugpub("IMU_Grav", &imuGrav);
unsigned long last_imu_pub_time = 0;


// Setup heartbeat publisher
std_msgs::Empty heartbeat_msg;
ros::Publisher heartbeat_pub("Heartbeat", &heartbeat_msg);
unsigned long last_heartbeat_time = 0;

void setup() {
  // Set up communication
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(500000);


  // Initialize ROS node
  nh.initNode();

  // Advertise publishers and subscribers
  nh.advertise(heartbeat_pub);
  nh.advertise(imubpub);
  nh.advertise(imugpub);


  // Initialize IMU
  if (!IMU.begin())
  {
    nh.logerror("Could not find a valid BNO055 sensor, check wiring, address, sensor ID, and system status");
    while (1);
  }

  // Set up IMU  
  // Set mode
  /*
  We may want to start with OPERATION_MODE_COMPASS and see if it performs well.
  If we need more stability or faster response, we can switch to OPERATION_MODE_NDOF,
  keeping in mind that it might require more processing power and calibration effort.
  */
  //IMU.setMode(OPERATION_MODE_COMPASS);
  IMU.setMode(OPERATION_MODE_NDOF);

  // Upload calibration data
  calibrationData.accel_offset_x = 16908;
  calibrationData.accel_offset_y = 352;
  calibrationData.accel_offset_z = 256;
  calibrationData.gyro_offset_x = 20001;
  calibrationData.gyro_offset_y = 1;
  calibrationData.gyro_offset_z = 8564;
  calibrationData.mag_offset_z = 256;
  calibrationData.mag_offset_x = 20001;
  calibrationData.mag_offset_y = 1;
  calibrationData.accel_radius = 17409;
  calibrationData.mag_radius = 0;

  IMU.setSensorOffsets(calibrationData);

  // Use external crystal for better accuracy
  // Crystal must be configured AFTER loading calibration data into BNO055
  IMU.setExtCrystalUse(true);
}

void loop() {
   switch(event){
      case WAITING_FOR_NH:
        // If the node handle is connected, move to the next state
        if (nh.connected())
        {
          event = IMU_SETUP;
          nh.loginfo("Arduino node handle connected");
        }
        break;

      case IMU_SETUP:
        // If the IMU is fully calibrated, move to the next state
        if (IMU.isFullyCalibrated())
        {
          /* Display calibration status for each sensor. */
          uint8_t system, gyro, accel, mag = 0;
          IMU.getCalibration(&system, &gyro, &accel, &mag);
          
          char message[128]; // Assuming a maximum message length of 128 characters
          sprintf(message, "CALIBRATION: Sys=%d Gyro=%d Accel=%d Mag=%d", system, gyro, accel, mag);
          nh.logwarn(message);

          nh.loginfo("IMU is fully calibrated");
          event = TEST;
        }
        // If the IMU is not fully calibrated, run the calibration function
        else
        {
          nh.logwarn("IMU needs to be recalibrated");
          calibrateIMU();
        }
        break;

      case TEST:
        // Check if it's time to take a new IMU reading
        if (millis() - last_imu_pub_time >= IMU_READ_INTERVAL) {

          // Read IMU data
          IMU.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
          IMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

          // Prepare IMU data
          imuGrav.data = gravityData.acceleration.x * 10;
          imuBearing.data = orientationData.orientation.x;

          // Publish IMU data
          imubpub.publish(&imuBearing);
          imugpub.publish(&imuGrav);

          // Update last IMU publish time
          last_imu_pub_time = millis();
        }


     
      default: break;

    }
  

  // Publish heartbeat
  if (millis() - last_heartbeat_time > HEARTBEAT_PUB_INTERVAL) {
    heartbeat_pub.publish(&heartbeat_msg);
    last_heartbeat_time = millis();
  }


  // Handle ROS communication
  nh.spinOnce();
}

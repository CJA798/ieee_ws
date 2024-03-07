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
// create IMU object
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28, &Wire);
adafruit_bno055_offsets_t calibrationData;


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
  nh.loginfo("Heartbeat Publisher Initialized");



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
  IMU.setMode(OPERATION_MODE_COMPASS);
  nh.loginfo("IMU mode set to compass");

  // upload calibration data
  calibrationData.accel_offset_x = 16908;
  calibrationData.accel_offset_y = 306;
  calibrationData.accel_offset_z = 256;
  calibrationData.gyro_offset_x = 20001;
  calibrationData.gyro_offset_y = 1;
  calibrationData.gyro_offset_z = 8607;
  calibrationData.mag_offset_z = 256;
  calibrationData.mag_offset_x = 20001;
  calibrationData.mag_offset_y = 1;
  calibrationData.accel_radius = 17409;
  calibrationData.mag_radius = 4;

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
          event = 99;
        }
        // If the IMU is not fully calibrated, run the calibration function
        else
        {
          nh.logwarn("IMU needs to be recalibrated");
          calibrateIMU();
        }
        break;


     
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

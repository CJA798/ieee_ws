#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "macros.h"

ros::NodeHandle nh;

// Setup heartbeat publisher
std_msgs::Empty heartbeat_msg;
ros::Publisher heartbeat_pub("Heartbeat", &heartbeat_msg);
unsigned long last_heartbeat_time = 0;

void setup() {
  // Initialize ROS node
  nh.initNode();

  // Advertise publishers and subscribers
  nh.advertise(heartbeat_pub);
}

void loop() {


  
  // Publish heartbeat
  if (millis() - last_heartbeat_time > HEARTBEAT_PUB_INTERVAL) {
    heartbeat_pub.publish(&heartbeat_msg);
    last_heartbeat_time = millis();
  }


  // Handle ROS communication
  nh.spinOnce();
}

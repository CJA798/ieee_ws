#include <string>
#include <cstring>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
//#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

// global variables for wheel speed and robot state
float wheelSpeedOne;
float wheelSpeedTwo;
float wheelSpeedThree;
//std_msgs::Float64MultiArray wheelSpeeds;
std_msgs::Float32MultiArray wheelSpeeds;
std_msgs::String navState;
std_msgs::String botState;

// global variables for sensor data
int tofFront;
int tofLeft;
int tofRight;
int tofBack;
int bearing;
int gravVector;


// create subscriber callbacks
void tofOneCallback(const std_msgs::Int16::ConstPtr& msg){
    tofFront = msg->data;
}

void tofTwoCallback(const std_msgs::Int16::ConstPtr& msg){
    tofLeft = msg->data;
}

void tofThreeCallback(const std_msgs::Int16::ConstPtr& msg){
    tofRight = msg->data;
}

void tofFourCallback(const std_msgs::Int16::ConstPtr& msg){
    tofBack = msg->data;
}

void imuBearCallback(const std_msgs::Int16::ConstPtr& msg){
    bearing = msg->data;
}

void imuGravCallback(const std_msgs::Int16::ConstPtr& msg){
    gravVector = msg->data;
}

void stateCallback(const std_msgs::String::ConstPtr& msg){
    botState.data = msg->data;
}


int main(int argc, char **argv) {
    // Initialize ROS node and name node
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle nh;

    // create subscriber objects
    ros::Subscriber front_tof_sub = nh.subscribe("TOF_Front", 10, tofOneCallback);
    ros::Subscriber left_tof_sub = nh.subscribe("TOF_Left", 10, tofTwoCallback);
    ros::Subscriber right_tof_sub = nh.subscribe("TOF_Right", 10, tofThreeCallback);
    ros::Subscriber back_tof_sub = nh.subscribe("TOF_Left", 10, tofFourCallback);
    ros::Subscriber bearing_sub = nh.subscribe("IMU_Bearing", 10, imuBearCallback);
    ros::Subscriber grav_sub = nh.subscribe("IMU_Grav", 10, imuGravCallback);
    ros::Subscriber bot_state_sub = nh.subscribe("State", 10, stateCallback);

    // create publisher objects
    ros::Publisher nav_state_pub = nh.advertise<std_msgs::String>("State_Status", 10);
    //ros::Publisher wheel_speed_pub = nh.advertise<std_msgs::Float64MultiArray>("wheel_speeds", 10);
    ros::Publisher wheel_speed_pub = nh.advertise<std_msgs::Float32MultiArray>("Wheel_Speeds", 10);

    // Set the loop rate
    ros::Rate loopRate(10); // 10 Hz

    while (ros::ok()) {
        // publish
        navState.data = "going";
        wheelSpeeds.data = {wheelSpeedOne, wheelSpeedTwo, wheelSpeedThree};
        nav_state_pub.publish(navState);
        wheel_speed_pub.publish(wheelSpeeds);

        // Spin and sleep
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
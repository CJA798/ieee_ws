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
std_msgs::Float64MultiArray wheelSpeeds;
std_msgs::String navState;
std_msgs::String botState;

// global variables for sensor data
int tofFront;
int tofLeft;
int tofRight;
int tofBack;
int bearing;
int gravVector;


<<<<<<< HEAD
class RobotController {
public:
    RobotController() {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Publishers
        stateStatusPub = nh.advertise<std_msgs::String>("state_status", 10);
        wheelSpeedsPub = nh.advertise<std_msgs::Float32MultiArray>("wheel_speeds", 10);

        // Subscribers
        tofFrontSub = nh.subscribe("TOF_Front", 10, &RobotController::tofFrontCallback);
        tofLeftSub = nh.subscribe("TOF_Left", 10, &RobotController::tofLeftCallback);
        tofRightSub = nh.subscribe("TOF_Right", 10, &RobotController::tofRightCallback);
        tofBackSub = nh.subscribe("TOF_Back", 10, &RobotController::tofBackCallback);
        imuBearingSub = nh.subscribe("IMU_Bearing", 10, &RobotController::imuBearingCallback);
        imuGravSub = nh.subscribe("IMU_Grav", 10, &RobotController::imuGravCallback);
        stateSub = nh.subscribe("State", 10, &RobotController::stateCallback);
    }

    void tofFrontCallback(const std_msgs::Int16::ConstPtr& msg) {
        tofFront = msg->data;
    }

    void tofLeftCallback(const std_msgs::Int16::ConstPtr& msg) {
        tofLeft = msg->data;
    }

    void tofRightCallback(const std_msgs::Int16::ConstPtr& msg) {
        tofRight = msg->data;
    }
=======
// create subscriber callbacks
void tofOneCallback(const std_msgs::Int16::ConstPtr& msg){
    tofFront = msg->data;
}
>>>>>>> 17271a5f22e66f84eed4c9a2019af782a6ae3557

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

<<<<<<< HEAD
        // Publishing data to wheel speeds topic
        std_msgs::Float32MultiArray wheelSpeeds;
        wheelSpeeds.data = {wheelSpeedOne, wheelSpeedTwo, wheelSpeedThree}; 
        wheelSpeedsPub.publish(wheelSpeeds);
    }
=======
void stateCallback(const std_msgs::String::ConstPtr& msg){
    botState.data = msg->data;
}
>>>>>>> 17271a5f22e66f84eed4c9a2019af782a6ae3557


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
    ros::Publisher nav_state_pub = nh.advertise<std_msgs::String>("state_status", 10);
    ros::Publisher wheel_speed_pub = nh.advertise<std_msgs::Float64MultiArray>("wheel_speeds", 10);

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
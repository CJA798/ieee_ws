#include <string>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
//#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

// global variables for wheel speed and robot state
float wheelSpeedOne;
float wheelSpeedTwo;
float wheelSpeedThree;
string navState;
char botState;

// global variables for sensor data
int tofFront;
int tofLeft;
int tofRight;
int tofBack;
int bearing;
int gravVector;


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

    void tofBackCallback(const std_msgs::Int16::ConstPtr& msg) {
        tofBack = msg->data;
    }

    void imuBearingCallback(const std_msgs::Int16::ConstPtr& msg) {
        bearing = msg->data;
    }

    void imuGravCallback(const std_msgs::Int16::ConstPtr& msg) {
        gravVector = msg->data;
    }

    void stateCallback(const std_msgs::String::ConstPtr& msg) {
        botState = msg->data;
    }

    void publishData() {
        // Publishing data to state status topic
        std_msgs::String stateStatus;
        stateStatus.data = navState;
        stateStatusPub.publish(stateStatus);

        // Publishing data to wheel speeds topic
        std_msgs::Float32MultiArray wheelSpeeds;
        wheelSpeeds.data = {wheelSpeedOne, wheelSpeedTwo, wheelSpeedThree}; 
        wheelSpeedsPub.publish(wheelSpeeds);
    }

private:
    ros::Publisher stateStatusPub;
    ros::Publisher wheelSpeedsPub;
    ros::Subscriber tofFrontSub;
    ros::Subscriber tofLeftSub;
    ros::Subscriber tofRightSub;
    ros::Subscriber tofBackSub;
    ros::Subscriber imuBearingSub;
    ros::Subscriber imuGravSub;
    ros::Subscriber stateSub;
};

int main(int argc, char **argv) {
    // Initialize ROS node and name node
    ros::init(argc, argv, "robot_controller");

    // Create an instance of the RobotController class
    RobotController robotController;

    // Set the loop rate
    ros::Rate loopRate(10); // 10 Hz

    while (ros::ok()) {
        // Call any other logic or functionality here
        // ...

        // Publish data to topics
        robotController.publishData();

        // Spin and sleep
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
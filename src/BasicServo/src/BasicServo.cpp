/*
* servos.cpp
* Chris Bass
* 2/4/24
*
* This program takes input from ros topics and writes to dynamixel servos,
* it provides a arm moving state and feed back of arm torque,
* it also accepts a Move function and does the kinematics and PIDs
*/

#include <ros/ros.h>
#include <math.h>
#include <unistd.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"

// Main class holding all servo declarations, functions, and variables
class BasicServoClass{
public:
    // Int flags for main
        int arm_moving = 0, misc_moving = 0;

    // Initiating pubs, subs, and arrays
    BasicServoClass(ros::NodeHandle* nodehandle){ 
        nh = *nodehandle;       

        // Resize Publisher Arrays
        Arm_Angles.data.resize(9);
        Move.data.resize(5);
        Misc_Angles_pub.data.resize(8);

        // Publishers
        Move_pub = nh.advertise<std_msgs::Float32MultiArray>("/Move", 1);
        Arm_Angles_pub = nh.advertise<std_msgs::Float32MultiArray>("/Arm_Angles", 1);
        Misc_Angles_pub = nh.advertise<std_msgs::Float32MultiArray>("/Misc_Angles_pub", 1);

        // Subscribers
        Move_Done_sub = nh.subscribe("/Move_Done", 1, &ServoClass::Move_DoneCallback, this);
        Arm_Done_sub = nh.subscribe("/Arm_Done", 1, &ServoClass::Arm_DoneCallback, this);
        Misc_Done_sub = nh.subscribe("/Misc_Done", 1, &ServoClass::Misc_DoneCallback, this);
        TOF_Back_sub = nh.subscribe("/TOF_Back", 1, &ServoClass::TOF_BackCallback, this);
        IMU_Grav_sub = nh.subscribe("/IMU_Grav", 1, &ServoClass::IMU_GravCallback, this);
        LED_State_sub = nh.subscribe("/LED_State", 1, &ServoClass::LED_StateCallback, this);
    }

    void Move_DoneCallback(const std_msgs::Float32MultiArray& Move_Done){

    }


// Pub, sub, and ros declarations
private: 
    ros::NodeHandle nh;

    // Publisher ros declarations
    ros::Publisher Move_pub;
    ros::Publisher Arm_Angles_pub;
    ros::Publisher Misc_Angles_pub;

    // Subscriber ros declarations
    ros::Subscriber Move_Done_sub;
    ros::Subscriber Arm_Done_sub;
    ros::Subscriber Misc_Done_sub;
    ros::Subscriber TOF_Back_sub;
    ros::Subscriber IMU_Grav_sub;
    ros::Subscriber LED_State_sub;


    // Publisher variable declarations
    std_msgs::Float32MultiArray Move;
    std_msgs::Float32MultiArray Arm_Angles;
    std_msgs::Float32MultiArray Misc_Angles;
};



// Main loop
int main(int argc, char** argv){

    // Sets up ros node and stuff
    ros::init(argc, argv, "BasicServo");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);   // Do i still need this???

    // Create main servo class
    BasicServoClass BasicServoObject(&nh);

    // Main loop
    while (ros::ok())
    {
        // Once triggered by new arm posistion will keep running until close enough to desired posistion
        if(BasicServoObject.arm_moving)
            BasicServoObject.armMoving();
        

        ros::spinOnce();
        //spinner.spin();
    }
    return 0;
}

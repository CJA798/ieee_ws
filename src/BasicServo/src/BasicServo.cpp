/*
* servos.cpp
* Chris Bass
* 2/4/24
*
* This program takes input from ros topics and writes to dynamixel servos
* it also provides a arm moving state and feed back of arm torque.
*/

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"

// Main class holding all servo declarations, functions, and variables
class ServoClass {
public:
    // Initiating pubs, subs, and arrays
    ServoClass(ros::NodeHandle* nodehandle) {
        nh = *nodehandle;

        // Resize Publisher Arrays
        Arm_Angles.data.resize(8);
        Task_Space.data.resize(5);

        // Publishers
        Arm_Angles_pub = nh.advertise<std_msgs::Float32MultiArray>("/Arm_Angles", 1);

        Task_Space_pub = nh.advertise<std_msgs::Float32MultiArray>("/Task_Space", 1);

        // Subscriber

        trigger_sub = nh.subscribe("/trigger", 1, &ServoClass::triggerCallback, this);
    }
    void triggerCallback(const std_msgs::Int32& trigger)
    {
        uint32_t temp = uint32_t(trigger.data);
        cord1[2] = temp % 1000;
        temp = temp / 1000;
        cord1[1] = temp % 1000;
        temp = temp / 1000;
        cord1[0] = temp % 1000;
        moveArm1(cord1);
        /*
        switch (state)
        {
        case 0:
            moveArm(pos1);
            state++;
            break;
        case 1:
            moveArm(pos2);
            state++;
            break;
        case 2:
            moveArm(pos3);
            state++;
            break;
        case 3:
            moveArm(pos4);
            state++;
            break;
            break;
        case 4:
            moveArm(pos5);
            state++;
            break;
        case 5:
        moveArm1(cord1);
            state = 4;
            break;
        default:
            break;
        }
        */
    }

    void moveArm(float pos[8])
    {
        for (int i = 0; i < 8; i++)
        {
            Arm_Angles.data[i] = pos[i] * 180 / M_PI * 11.3778 + 2048;
        }
        Arm_Angles_pub.publish(Arm_Angles);
    }

    void moveArm1(float cord[5])
    {
        for (int i = 0; i < 5; i++)
        {
            Task_Space.data[i] = cord[i];
        }
        Task_Space_pub.publish(Task_Space);
    }


    // Pub, sub, and ros declarations
private:
    ros::NodeHandle nh;

    // Publisher ros declarations
    ros::Publisher Task_Space_pub;
    ros::Publisher Arm_Angles_pub;

    // Subscriber ros declarations
    ros::Subscriber trigger_sub;

    // Publisher variable declarations
    std_msgs::Float32MultiArray Task_Space;
    std_msgs::Float32MultiArray Arm_Angles;

    int state = 0;

    float pos1[8] = { 0.000000, -0.216435, -0.216435, 1.108021, 0.000000, -2.462383, 0.000000, -0.500000 },
        pos2[8] = { -0.221851, -0.024442, -0.024442, -0.721155, 0.000000, -0.825199, -0.000000, -0.500000 },
        pos3[8] = { -0.290833, -0.296124, -0.296124, -0.715075, 0.000000, -0.559597, -0.000000, -0.500000 },
        pos4[8] = { -0.290833, -0.296124, -0.296124, -0.715075, 0.000000, -0.559597, -0.000000, 0.128854 },
        pos5[8] = { -0.000000, -0.000000, -0.000000, -0.000000, 0.000000, -1.570796, -0.000000, 0.128854 };

    float cord1[5] = {100, 100, 100, 2048, 2048};
};



// Main loop
int main(int argc, char** argv) {

    // Sets up ros node and stuff
    ros::init(argc, argv, "BasicServo");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);   // Do i still need this???

    // Create main servo class
    ServoClass ServoObject(&nh);

    // Main loop
    while (ros::ok())
    {
        // Once triggered by new arm posistion will keep running until close enough to desired posistion

        ros::spinOnce();
        //spinner.spin();
    }

    return 0;
}
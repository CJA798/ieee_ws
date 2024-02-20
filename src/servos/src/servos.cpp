/*
* servos.cpp
* Chris Bass
* 2/4/24
*
* This program takes input from ros topics and writes to dynamixel servos
* it also provides a arm moving state and feed back of arm torque.
*/

#include <ros/ros.h>
#include <math.h>
#include <unistd.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"

using namespace dynamixel;

// Address table of servos
#define TORQUE_ENABLE_ADDR    64
#define GOAL_POSITION_ADDR   116
#define MAX_VEL_ADDR 112
#define MAX_ACC_ADDR 108
#define PRESENT_POSITION_ADDR 132
#define GOAL_VELOCITY_ADDR   104
#define POS_P_GAIN_ADDR 84
#define POS_I_GAIN_ADDR 82
#define POS_D_GAIN_ADDR 80

// Defined values
#define ARM_SECONDARY_ID  100   // Secondary ID for writing multiple servos at once
#define MAX_VEL 1000//100             // Max velocity of arm joints
#define MAX_ACC 200//2               // Max acceleration of arm joints
#define TORQUE_ENABLE 1         // Enable torque on servos
#define CENTER_POSISTION  2048  // Center posistion for angle mode on servos
#define POS_P_GAIN  640         // Angle mode P gain
#define POS_I_GAIN  0           // Angle mode I gain
#define POS_D_GAIN  4000        // Angle mode D gain
#define STOP  0                 // Velocity mode stop

// PID's
#define KP_X    0.1;
#define KI_X    0.0;
#define KD_X    0.0;

#define KP_Y    0.1;
#define KI_Y    0.0;
#define KD_Y    0.0;

#define KP_Z    0.1;
#define KI_Z    0.0;
#define KD_Z    0.0;

#define TS      10;

// Default dynamixel setting
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT89FKZ2-if00-port0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Set up dynamixel ports and packets
PortHandler* portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler* packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Create objects for bult reading and writing of servos
GroupBulkRead groupBulkRead(portHandler, packetHandler);
GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

// Global variables
int arm_moving = 0;     // Keeps track of weather or not the arm need to be check for moving

// Main class holding all servo declarations, functions, and variables
class ServoClass{
public:
    // Initiating pubs, subs, and arrays
    ServoClass(ros::NodeHandle* nodehandle){ 
        nh = *nodehandle;        

        // Resize Publisher Arrays
        Arm_Angles.data.resize(9);
        Feedback.data.resize(8);
        Wheel_Speeds.data.resize(3);

        // Publishers
        Feedback_pub = nh.advertise<std_msgs::Float32MultiArray>("/Feedback", 1);
        Arm_Angles_pub = nh.advertise<std_msgs::Float32MultiArray>("/Arm_Angles", 1);
        Arm_Done_pub = nh.advertise<std_msgs::Int8>("/Arm_Done", 1);
        Move_Done_pub = nh.advertise<std_msgs::Int8>("/Move_Done", 1);
        Wheel_Speeds_pub = nh.advertise<std_msgs::Float32MultiArray>("/Wheel_Speeds", 1);

        // Subscribers
        Get_Feedback_sub = nh.subscribe("/Get_Feedback", 1, &ServoClass::Get_FeedbackCallback, this);
        Task_Space_sub = nh.subscribe("/Task_Space", 1, &ServoClass::Task_SpaceCallback, this);
        Misc_Angles_sub = nh.subscribe("/Misc_Angles", 1, &ServoClass::Misc_AnglesCallback, this);
        Wheel_Speeds_sub = nh.subscribe("/Wheel_Speeds", 1, &ServoClass::Wheel_SpeedsCallback, this);
        Arm_Angles_sub = nh.subscribe("/Arm_Angles", 1, &ServoClass::Arm_AnglesCallback, this);
        Move_sub = nh.subscribe("/Move", 1, &ServoClass::MoveCallback, this);
        TOF_Front_sub = nh.subscribe("/TOF_Front", 1, &ServoClass::TOF_FrontCallback, this);
        TOF_Left_sub = nh.subscribe("/TOF_Left", 1, &ServoClass::TOF_LeftCallback, this);
        TOF_Right_sub = nh.subscribe("/TOF_Right", 1, &ServoClass::TOF_RightCallback, this);
        IMU_Bearing_sub = nh.subscribe("/IMU_Bearing", 1, &ServoClass::IMU_BearingCallback, this);
    }


    // Open servo ports, and set initial values
    void initializeServos(int arm) {
        // Starts communications with servos
        portHandler->openPort();            // Opens port to U2D2
        portHandler->setBaudRate(BAUDRATE); // Sets default baud rate

        // Sets up initial values and states for servos
        if (arm) {
            packetHandler->write1ByteTxOnly(portHandler, ARM_SECONDARY_ID, TORQUE_ENABLE_ADDR, TORQUE_ENABLE);      // Enable torque
            packetHandler->write1ByteTxOnly(portHandler, ARM_SECONDARY_ID, 65, 1);      // Turnn on LED
            packetHandler->write4ByteTxOnly(portHandler, ARM_SECONDARY_ID, MAX_ACC_ADDR, MAX_ACC / 100);                  // Set acc limit of posistion mode
            packetHandler->write4ByteTxOnly(portHandler, ARM_SECONDARY_ID, MAX_VEL_ADDR, MAX_VEL / 10);                  // Set vel limit of posistion mode
            packetHandler->write4ByteTxOnly(portHandler, ARM_SECONDARY_ID, GOAL_VELOCITY_ADDR, STOP);               // Sets starting velocity to stop
            packetHandler->write4ByteTxOnly(portHandler, ARM_SECONDARY_ID, GOAL_POSITION_ADDR, CENTER_POSISTION);   // Set starting posistion to center
            packetHandler->write2ByteTxOnly(portHandler, ARM_SECONDARY_ID, POS_P_GAIN_ADDR, POS_P_GAIN);            // Sets posistion P gain
            packetHandler->write2ByteTxOnly(portHandler, ARM_SECONDARY_ID, POS_I_GAIN_ADDR, POS_I_GAIN);            // Sets posistion I gain
            packetHandler->write2ByteTxOnly(portHandler, ARM_SECONDARY_ID, POS_D_GAIN_ADDR, POS_D_GAIN);            // Sets posistion D gain

            // Publish 8 starting servo angles and speed to Arm_Angles
            int Arm_Start_Angles[9] = { 2069, 2814, 2808, 1533, 2058, 884, 2134, 2050, 1 };
            for (int i = 0; i < 9; i++) {
                Arm_Angles.data[i] = Arm_Start_Angles[i];
            }
            Arm_Angles_pub.publish(Arm_Angles);
        }
    }


    // Takes global x, y, z, theta(wrist), phi(claw), and speed and pubs servo values to Arm_Angles
    void Task_SpaceCallback(const std_msgs::Float32MultiArray& Task_Space){
        // Declares variables and copies taskspace from topic
        float x, y, z, theta, phi;
        x = Task_Space.data[0];
        y = Task_Space.data[1];
        z = Task_Space.data[2];
        theta = Task_Space.data[3];
        phi = Task_Space.data[4];

        //Print x, y, z recieved to ros
        //ROS_INFO("%d, %d, %d, %d, %d]", (int)x, (int)y, (int)z, (int)theta, (int)phi);

        // Creat arrays for calculations and predefined values
        float q[6] = { 0, 0, 0, 0, 0, 0 };                          // Major joint angles of robot
        int Q[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };                      // Final joint angles for servos including duplicated j2 and claw
        float l[9] = { 32.5, 162, 24, 24, 148.5, 75.34, 17, 0, 0 }; // Predefined lengts of links for DH params

        // Reverse kinematics math
        float x1 = sqrt(x * x + z * z) + l[6] - l[3];
        float y1 = y - l[0] + l[5];
        l[7] = sqrt(l[2] * l[2] + l[4] * l[4]);
        l[8] = sqrt(x1 * x1 + y1 * y1);
        float beta = acos((l[1] * l[1] + l[8] * l[8] - l[7] * l[7]) / (2 * l[1] * l[8]));
        float alpha = acos((l[7] * l[7] + l[1] * l[1] - l[8] * l[8]) / (2 * l[7] * l[1]));
        float omega = acos(l[2] / l[7]);
        float phi2 = M_PI / 2 - atan(y1 / x1);

        // Major joint angles of robot in radians
        q[0] = atan2(-z, x);
        q[1] = beta - phi2;
        q[2] = alpha + omega - M_PI;
        q[3] = -q[1] - q[2] - M_PI / 2;
        q[4] = theta;
        q[5] = (phi - 2048) * M_PI / 180 / 11.3778;

        //Final servo angles including duplicated J2 and claw in int's 0 - 4096 centered around 2048
        Q[0] = (int)(q[0] * 180 / M_PI * 11.3778 + 2048);
        Q[1] = (int)(q[1] * 180 / M_PI * 11.3778 + 2048);
        Q[2] = Q[1];
        Q[3] = (int)(q[2] * 180 / M_PI * 11.3778 + 2048);
        Q[4] = 2048;
        Q[5] = (int)(q[3] * 180 / M_PI * 11.3778 + 2048);
        Q[6] = (int)theta;
        Q[7] = (int)phi;
        Q[8] = Task_Space.data[5];

        // Print final servo values to ros
        //ROS_INFO("%d, %d, %d, %d, %d, %d", (int)(q[0] * 180 / M_PI), (int)(q[1] * 180 / M_PI), (int)(q[2] * 180 / M_PI), (int)(q[3] * 180 / M_PI), (int)(q[4] * 180 / M_PI), (int)(q[5] * 180 / M_PI));
        
        // Publish 8 servo angles and speed to Arm_Angles
        for (int i = 0; i < 9; i++){
            Arm_Angles.data[i] = Q[i];
        }
        Arm_Angles_pub.publish(Arm_Angles);
    }


    // Takes 8 Arm_Angles and speed and writes to servos ID 1-8
    void Arm_AnglesCallback(const std_msgs::Float32MultiArray& Arm_Angles){
        // Write accel and speed for servos of arm
        armSpeed(Arm_Angles.data[8]);

        // Clears bulk write stack
        groupBulkWrite.clearParam();

        // Creates and assigns array with each byte of message
        uint8_t data_array[4];
        for (int i = 1; i < 9; i++){    // Do for all 8 arm servos
            uint32_t data = (unsigned int)(Arm_Angles.data[i - 1]); // Convert int32 to uint32
            data_array[0] = DXL_LOBYTE(DXL_LOWORD(data));
            data_array[1] = DXL_HIBYTE(DXL_LOWORD(data));
            data_array[2] = DXL_LOBYTE(DXL_HIWORD(data));
            data_array[3] = DXL_HIBYTE(DXL_HIWORD(data));
            groupBulkWrite.addParam(i, GOAL_POSITION_ADDR, 4, data_array);  // Adds message to stack arguments(servo ID, Address, size, data array of bytes)
        }
        groupBulkWrite.txPacket();  // Write servos with prepared list all at once

        // Sets the arm moving flag to start checking for arm to complete move
        arm_moving = 1;

        // Print final servo values to ros
        //ROS_INFO("Angles: %f, %f, %f, %f, %f, %f, %f, %f", Arm_Angles.data[0], Arm_Angles.data[1], Arm_Angles.data[2], Arm_Angles.data[3], Arm_Angles.data[4], Arm_Angles.data[5], Arm_Angles.data[6], Arm_Angles.data[7]);

        /* Alternate way of writing servos without bulkwrite
        for (int i = 1; i < 9; i++) {
          int temp = (int)(arm_angles.data[i - 1] * 180 / M_PI * 11.3778 + 2048);
          packetHandler->write4ByteTxOnly(portHandler, i, GOAL_POSITION_ADDR, temp);
          usleep(10000);
        }
        */
    }


    // Takes 3 Wheel_Speeds and writes to servos ID 9-11
    void Wheel_SpeedsCallback(const std_msgs::Float32MultiArray& Wheel_Speeds){
        // Clears bulk write stack
        groupBulkWrite.clearParam();

        // Creates and assigns array with each byte of message
        uint8_t data_array[4];
        for (int i = 1; i < 4; i++){    // Do for all 3 wheel servos
            int32_t data = (int32_t)(Wheel_Speeds.data[i - 1]); // Convert int32 to uint32
            data_array[0] = DXL_LOBYTE(DXL_LOWORD(data));
            data_array[1] = DXL_HIBYTE(DXL_LOWORD(data));
            data_array[2] = DXL_LOBYTE(DXL_HIWORD(data));
            data_array[3] = DXL_HIBYTE(DXL_HIWORD(data));
            groupBulkWrite.addParam((i + 8), GOAL_VELOCITY_ADDR, 4, data_array);  // Adds message to stack arguments(servo ID, Address, size, data array of bytes)
        }
        groupBulkWrite.txPacket();  // Write servos with prepared list all at once

        // Print final servo values to ros
        //ROS_INFO("Angles: %f, %f, %f", Wheel_Speeds.data[0], Wheel_Speeds.data[1], Wheel_Speeds.data[2]);
    }


    // Takes 8 Misc_Angles and writes to servos ID 12-21
    void Misc_AnglesCallback(const std_msgs::Float32MultiArray& Misc_Angles){
        // Clears bulk write stack
        groupBulkWrite.clearParam();

        // Creates and assigns array with each byte of message
        uint8_t data_array[4];
        for (int i = 1; i < 9; i++){     // Do for all 8ish misc servos
            uint32_t data = (unsigned int)(Misc_Angles.data[i - 1]); // Convert int32 to uint32
            data_array[0] = DXL_LOBYTE(DXL_LOWORD(data));
            data_array[1] = DXL_HIBYTE(DXL_LOWORD(data));
            data_array[2] = DXL_LOBYTE(DXL_HIWORD(data));
            data_array[3] = DXL_HIBYTE(DXL_HIWORD(data));
            groupBulkWrite.addParam((i + 11), GOAL_POSITION_ADDR, 4, data_array);  // Adds message to stack arguments(servo ID, Address, size, data array of bytes)
        }
        groupBulkWrite.txPacket();  // Write servos with prepared list all at once

        // Print final servo values to ros
        //ROS_INFO("Angles: %f, %f, %f, %f, %f, %f, %f, %f", Misc_Angles.data[0], Misc_Angles.data[1], Misc_Angles.data[2], Misc_Angles.data[3], Misc_Angles.data[4], Misc_Angles.data[5], Misc_Angles.data[6], Misc_Angles.data[7]);
    }

    //*******// Curently sending arm angles instead
    // Read present loads and publishes to feedback
    void Get_FeedbackCallback(const std_msgs::Int8& Get_Feedback){
        // Clears bulk read stack
        groupBulkRead.clearParam();

        // Reads servo torques of arm and stores in array
        for (int i = 1; i < 9; i++){
            //********// groupBulkRead.addParam(i, 126, 2);
            groupBulkRead.addParam(i, 132, 4);
        }
        groupBulkRead.txRxPacket(); // Executes bulk read

        // Assigns temp bulk read array to topic and publishes
        for (int i = 0; i < 8; i++){
            //**********//Feedback.data[i] = groupBulkRead.getData((i + 1), 126, 2);
            Feedback.data[i] = groupBulkRead.getData((i + 1), 132, 4);
        }
        Feedback_pub.publish(Feedback); // Publishes 8x array of arm servo torques

        // Prints arm servo torques
        //ROS_INFO("Present Load: %d, %d, %d, %d, %d, %d, %d, %d", Feedback.data[0], Feedback.data[1], Feedback.data[2], Feedback.data[3], Feedback.data[4], Feedback.data[5], Feedback.data[6], Feedback.data[7]);
    }


    // Checks if robot arm is within exceptable error to declare it has arrived
    void armMoving(){
        // Clears bulk read stack
        groupBulkRead.clearParam();

        // Reads servo posistions of arm and stores in array
        for (int i = 1; i < 9; i++){
            groupBulkRead.addParam(i, 132, 4);
        }
        groupBulkRead.txRxPacket(); // Executes bulk read

        // Checks if current posistion and goal posistion are withen acceptable toleracne
        for (int i = 0; i < 8; i++){
            if(abs(Arm_Angles.data[i] - groupBulkRead.getData((i + 1), 132, 4)) > 50){ // Used to use uint32_t pos[8]; for storing
                arm_moving = 1;                 // Joint error is to large que up another check
                Arm_Done.data = 0;              //  mark arm and not done moving
                Arm_Done_pub.publish(Arm_Done); //  publish still moving result
                return;                         //  break out of function
            }
        }
        arm_moving = 0;                 // All joints in acceptable range so remove funtion for loop
        Arm_Done.data = 1;              //  mark arm as done moving
        Arm_Done_pub.publish(Arm_Done); //  publish done moving result
    }


    // Writes accels and velocity to the servos
    void armSpeed(int speed){
        // Clears bulk write stack
        groupBulkWrite.clearParam();

        // Scales speed of vel and acc
        uint32_t max_acc = (uint32_t)(MAX_ACC * speed / 100);
        uint32_t max_vel = (uint32_t)(MAX_VEL * speed / 100);

        // Creates and assigns array with each byte of messages
        uint8_t data_array_acc[4] = {DXL_LOBYTE(DXL_LOWORD(max_acc)), DXL_HIBYTE(DXL_LOWORD(max_acc)), DXL_LOBYTE(DXL_HIWORD(max_acc)), DXL_HIBYTE(DXL_HIWORD(max_acc))};
        uint8_t data_array_vel[4] = {DXL_LOBYTE(DXL_LOWORD(max_vel)), DXL_HIBYTE(DXL_LOWORD(max_vel)), DXL_LOBYTE(DXL_HIWORD(max_vel)), DXL_HIBYTE(DXL_HIWORD(max_vel))};
        
        // Adds messages to stack to write
        for (int i = 1; i < 9; i++){     // Do for all 8 servos
            groupBulkWrite.addParam((i), MAX_ACC_ADDR, 4, data_array_acc);  // Adds message to stack arguments(servo ID, Address, size, data array of bytes)
            groupBulkWrite.addParam((i), MAX_VEL_ADDR, 4, data_array_vel);  // Adds message to stack arguments(servo ID, Address, size, data array of bytes)
        }
        groupBulkWrite.txPacket();  // Write servos with prepared list all at once
    }


    // 
    void TOF_FrontCallback(const std_msgs::Int16& TOF_Front){
        if(desired_y == 0)
            linear_y = 0;
        else if(desired_y < 0)
            linear_y = TOF_Front.data;
        else{
            double error_y = desired_y - TOF_Front.data;
            /*
            if(error_y > 180)
                error_y -= 360;
            if(error_y < -180)
                error_y += 360;
            */
            error_y_cumulative += error_y;
            linear_y = (KP_Y * error_y) + (KI_Y * error_y_cumulative / TS) + (KD_Y * TS * (error_y - error_y_prev));
            error_y_prev = error_y;
        }
    }


    // 
    void TOF_LeftCallback(const std_msgs::Int16& TOF_Left){
        if(desired_x == 0)
            linear_x = 0;
        else if(desired_x < 0){
            double error_x = TOF_Left.data - desired_x;
            /*
            if(error_x > 180)
                error_x -= 360;
            if(error_x < -180)
                error_x += 360;
            */
            error_x_cumulative += error_x;
            linear_x = (KP_X * error_x) + (KI_X * error_x_cumulative / TS) + (KD_X * TS * (error_x - error_x_prev));
            error_x_prev = error_x;
        }
    }


    // 
    void TOF_RightCallback(const std_msgs::Int16& TOF_Right){
        if(desired_x == 0)
            linear_x = 0;
        else if(desired_x > 0){
            double error_x = desired_x - TOF_Right.data;
            /*
            if(error_x > 180)
                error_x -= 360;
            if(error_x < -180)
                error_x += 360;
            */
            error_x_cumulative += error_x;
            linear_x = (KP_X * error_x) + (KI_X * error_x_cumulative / TS) + (KD_X * TS * (error_x - error_x_prev));
            error_x_prev = error_x;
        }
    }


    // 
    void IMU_BearingCallback(const std_msgs::Int16& IMU_Bearing){
        double error_z = desired_z - IMU_Bearing.data;

        if(error_z > 180)
            error_z -= 360;
        if(error_z < -180)
            error_z += 360;

        error_z_cumulative += error_z;
        linear_z = (KP_Z * error_z) + (KI_Z * error_z_cumulative / TS) + (KD_Z * TS * (error_z - error_z_prev));
        error_z_prev = error_z;
    }


    // Accepts move instructions for bot x, y, z, speed and follows those values.
    void MoveCallback(const std_msgs::Float32MultiArray& Move){
        desired_x = Move.data[0];
        desired_y = Move.data[1];
        desired_z = Move.data[2];
        max_speed = Move.data[3];
    }


    // 
    void botKinematics(){
        // linear_xyz, max_speed

        double theta = atan(linear_y/linear_x);
        if(linear_x < 0 && linear_y >= 0)
            theta += 3.1415;
        else if(linear_x < 0 && linear_y < 0)
            theta += -3.1415;
        if(theta < 0)
            theta += 6.28;

        double speed = sqrt(linear_x^2 + linear_y^2);
        if(speed > max_speed)
            speed = max_speed;

        Wheel_Speeds.data[0] = speed * cos(theta) + linear_z;
        Wheel_Speeds.data[1] = speed * cos(theta + (2.0 / 3.0 * 3.1415)) + linear_z;
        Wheel_Speeds.data[2] = speed * cos(theta - (2.0 / 3.0 * 3.1415)) + linear_z;
    }


// Pub, sub, and ros declarations
private: 
    ros::NodeHandle nh;

    // Publisher ros declarations
    ros::Publisher Feedback_pub;
    ros::Publisher Arm_Angles_pub;
    ros::Publisher Arm_Done_pub;
    ros::Publisher Move_Done_pub;
    ros::Publisher Wheel_Speeds_pub;

    // Subscriber ros declarations
    ros::Subscriber Get_Feedback_sub;
    ros::Subscriber Task_Space_sub;
    ros::Subscriber Misc_Angles_sub;
    ros::Subscriber Wheel_Speeds_sub;
    ros::Subscriber Arm_Angles_sub;
    ros::Subscriber Move_sub;
    ros::Subscriber TOF_Front;
    ros::Subscriber TOF_Left;
    ros::Subscriber TOF_Right;
    ros::Subscriber IMU_Bearing;

    // Publisher variable declarations
    std_msgs::Float32MultiArray Feedback;
    std_msgs::Float32MultiArray Arm_Angles;
    std_msgs::Float32MultiArray Wheel_Speeds;
    std_msgs::Int8 Arm_Done;

    // Variables for functions
    double  desired_x = 0, error_x_prev = 0, error_x_cumulative = 0, linear_x = 0, 
            desired_y = 0, error_y_prev = 0, error_y_cumulative = 0, linear_y = 0, 
            desired_z = 0, error_z_prev = 0, error_z_cumulative = 0, linear_z = 0,
            max_speed = 0;
};



// Main loop
int main(int argc, char** argv){

    // Sets up ros node and stuff
    ros::init(argc, argv, "servos");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);   // Do i still need this???

    // Create main servo class
    ServoClass ServoObject(&nh);
    // Starts communication and sets start vaules for servos
    ServoObject.initializeServos(1);

    // Main loop
    while (ros::ok())
    {
        // Once triggered by new arm posistion will keep running until close enough to desired posistion
        if(arm_moving)
            ServoObject.armMoving();

        ros::spinOnce();
        //spinner.spin();
    }

    // Close ports and stop node
    packetHandler->write4ByteTxOnly(portHandler, ARM_SECONDARY_ID, GOAL_VELOCITY_ADDR, STOP);               // Sets velocity to stop
    portHandler->closePort();
    return 0;
}

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

using namespace dynamixel;

// Address table of servos
#define TORQUE_ENABLE_ADDR      64
#define GOAL_POSITION_ADDR      116
#define MAX_VEL_ADDR            112
#define MAX_ACC_ADDR            108
#define PRESENT_POSITION_ADDR   132
#define GOAL_VELOCITY_ADDR      104
#define POS_P_GAIN_ADDR         84
#define POS_I_GAIN_ADDR         82
#define POS_D_GAIN_ADDR         80

// Defined servo and arm values
#define ARM_SECONDARY_ID        100     // Secondary ID for writing multiple servos at once
#define MAX_VEL                 1000    //100    // Max velocity of arm joints
#define MAX_ACC                 200     //2      // Max acceleration of arm joints
#define TORQUE_ENABLE           1       // Enable torque on servos
#define CENTER_POSISTION        2048    // Center posistion for angle mode on servos
#define POS_P_GAIN              640     // Angle mode P gain
#define POS_I_GAIN              500     // Angle mode I gain
#define POS_D_GAIN              4000    // Angle mode D gain
#define STOP                    0       // Velocity mode stop
#define MISC_COUNT              1       // Number of misc servos hooked up
#define MISC_ANGLE_TOLERANCE    50      // Counts we need to be off to count as arrived
#define ARM_TOLERANCE           10      // Allowable error in arm before declareing arrived
#define STEPS_DOWN              5       // Number of millimeters to move down per step


#define CUSTOM_PIDS             1       // Set to 1 for custom variable pids, or 0 for preset

// PID's and Move constants x = left/right, y = forward/backwards, z = rotation
#if CUSTOM_PIDS// Custom PIDs for use with publishing
double KP_X = 4.0, KI_X = 0.01, KD_X = 0.5, KP_Y = 4.0, KI_Y = 0.01, KD_Y = 0.5, KP_Z = 6.0, KI_Z = 0.01, KD_Z = 0.3, ALLOWABLE_ERROR = 20, SEQUENTIAL_READS = 10;

#else // Preset PIDs
#define KP_X    4.0//4.0
#define KI_X    0.01//0.01
#define KD_X    0.5//0.5

#define KP_Y    4.0//4.0
#define KI_Y    0.01//0.01
#define KD_Y    0.5//0.5

#define KP_Z    6.0//6.0
#define KI_Z    0.01//0.01
#define KD_Z    0.3//0.3

#define ALLOWABLE_ERROR     20//20  // How close we need to be to the final posistion for each axis in mm
#define SEQUENTIAL_READS    10//10  // How many readings we need to be in these posistion before declaring arrival about 10 reading/sec
#endif

// Other PID and wheel constants
#define TS                  10      // Estimation of TOF samples per sec
#define MAX_SPEED           5       // Scaler for max_speed of wheels
#define CUMULATIVE_CAP      1000    // Cap on cumlative error for pid

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

// Main class holding all servo declarations, functions, and variables
class ServoClass{
public:
    // Int flags for main
        int arm_moving = 0, misc_moving = 0;

    // Initiating pubs, subs, and arrays
    ServoClass(ros::NodeHandle* nodehandle){ 
        nh = *nodehandle;       

        // Resize Publisher Arrays
        Arm_Angles.data.resize(9);
        Feedback.data.resize(8);
        Wheel_Speeds.data.resize(3);
        Task_Space.data.resize(6);

        // Publishers
        Feedback_pub = nh.advertise<std_msgs::Float32MultiArray>("/Feedback", 1);
        Arm_Angles_pub = nh.advertise<std_msgs::Float32MultiArray>("/Arm_Angles", 1);
        Arm_Done_pub = nh.advertise<std_msgs::Int8>("/Arm_Done", 1);
        Move_Done_pub = nh.advertise<std_msgs::Int8>("/Move_Done", 1);
        Wheel_Speeds_pub = nh.advertise<std_msgs::Float32MultiArray>("/Wheel_Speeds", 1);
        Misc_Done_pub = nh.advertise<std_msgs::Int8>("/Misc_Done", 1);
        Task_Space_pub = nh.advertise<std_msgs::Float32MultiArray>("/Task_Space", 1);

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
        PIDs_sub = nh.subscribe("/PIDs", 1, &ServoClass::PIDsCallback, this);
    }


    // PID callback funtion for seting pids on the fly
    void PIDsCallback(const std_msgs::Float32MultiArray& PIDs){
        #if CUSTOM_PIDS
        int i = 0;
        KP_X = PIDs.data[i++];
        KI_X = PIDs.data[i++];
        KD_X = PIDs.data[i++];

        KP_Y = PIDs.data[i++];
        KI_Y = PIDs.data[i++];
        KD_Y = PIDs.data[i++];

        KP_Z = PIDs.data[i++];
        KI_Z = PIDs.data[i++];
        KD_Z = PIDs.data[i++];

        ALLOWABLE_ERROR = PIDs.data[i++];
        SEQUENTIAL_READS = PIDs.data[i++];
        
        #else
            return;
        #endif
    }


    // Open servo ports, and set initial values
    void initializeServos(int arm) {
        // Starts communications with servos
        portHandler->openPort();            // Opens port to U2D2
        portHandler->setBaudRate(BAUDRATE); // Sets default baud rate

        // Sets up initial values and states for servos
        if (arm) {
            packetHandler->write1ByteTxOnly(portHandler, ARM_SECONDARY_ID, TORQUE_ENABLE_ADDR, TORQUE_ENABLE);      // Enable torque
            packetHandler->write1ByteTxOnly(portHandler, ARM_SECONDARY_ID, 65, 1);      // Turn on LED
            packetHandler->write4ByteTxOnly(portHandler, ARM_SECONDARY_ID, MAX_ACC_ADDR, MAX_ACC / 100);                  // Set acc limit of posistion mode
            packetHandler->write4ByteTxOnly(portHandler, ARM_SECONDARY_ID, MAX_VEL_ADDR, MAX_VEL / 10);                  // Set vel limit of posistion mode
            packetHandler->write4ByteTxOnly(portHandler, ARM_SECONDARY_ID, GOAL_VELOCITY_ADDR, STOP);               // Sets starting velocity to stop
            packetHandler->write4ByteTxOnly(portHandler, ARM_SECONDARY_ID, GOAL_POSITION_ADDR, CENTER_POSISTION);   // Set starting posistion to center
            packetHandler->write2ByteTxOnly(portHandler, ARM_SECONDARY_ID, POS_P_GAIN_ADDR, POS_P_GAIN);            // Sets posistion P gain
            packetHandler->write2ByteTxOnly(portHandler, ARM_SECONDARY_ID, POS_I_GAIN_ADDR, POS_I_GAIN);            // Sets posistion I gain
            packetHandler->write2ByteTxOnly(portHandler, ARM_SECONDARY_ID, POS_D_GAIN_ADDR, POS_D_GAIN);            // Sets posistion D gain

            // Publish 8 starting servo angles and speed to Arm_Angles
            int Arm_Start_Angles[9] = { 1586, 2902, 2898, 1471, 2063, 1802, 1041, 1935, 1 };
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

        // If speed was negative then slowly move down at STEPS_DOWN pace
        if(Task_Space.data[5] < 0){
            // Copy to local task space
            for(int i = 0; i < 6; i++)
                local_task_space[i] = Task_Space.data[i];

            // Move y down by STEPS_DOWN
            if(Task_Space.data[5] + STEPS_DOWN < 0){
                y -= STEPS_DOWN;
                local_task_space[5] += STEPS_DOWN;
            }

            // Move y down to final destination
            else{
                y += Task_Space.data[5];
                local_task_space[5] = 1;
            }

            // Update y value
            local_task_space[1] = y;
        }

        //Print x, y, z recieved to ros
        //ROS_INFO("%d, %d, %d, %d, %d]", (int)x, (int)y, (int)z, (int)theta, (int)phi);

        // Creat arrays for calculations and predefined values
        float q[6] = { 0, 0, 0, 0, 0, 0 };                          // Major joint angles of robot
        int Q[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };                      // Final joint angles for servos including duplicated j2 and claw
        //float l[9] = { 32.5, 162, 24, 24, 148.5, 150, 53, 0, 0 }; // Predefined lengths of links for jaws attachment DH params
        //float l[9] = { 32.5, 162, 24, 24, 148.5, 82.5, 22.5, 0, 0 }; // Predefined lengths of links for new claw DH params
        //float l[9] = { 32.5, 162, 24, 24, 148.5, 75.34, 17, 0, 0 }; // Predefined lengths of links for old claw DH params
        float l[9] = { 32.5, 162, 24, 24, 148.5, 140, 18.4, 0, 0 }; // Predefined lengths of links for jaws attachment DH params

        // Inverse kinematics math
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
            if(abs(Arm_Angles.data[i] - groupBulkRead.getData((i + 1), 132, 4)) > ARM_TOLERANCE){ // Used to use uint32_t pos[8]; for storing
                arm_moving = 1;                 // Joint error is to large que up another check
                Arm_Done.data = 0;              //  mark arm as not done moving
                Arm_Done_pub.publish(Arm_Done); //  publish not done moving result
                return;                         //  break out of function
            }
        }

        // If moving down slowly, don't post arm done
        if(local_task_space[5] < 0){
            // Copy local variable and pub
            for(int i = 0; i < 6; i++)
                Task_Space.data[i] = local_task_space[i];
            Task_Space_pub.publish(Task_Space); 
        }
        else{
            arm_moving = 0;                 // All joints in acceptable range so remove funtion for loop
            Arm_Done.data = 1;              //  mark arm as done moving
            Arm_Done_pub.publish(Arm_Done); //  publish done moving result
        }
    }


    // Writes accels and velocity to the servos
    void armSpeed(int speed){
        // If negative value due to moveDown, move at slowest speed of 1
        if(speed <= 0)
            speed = 1;

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


    // Read present loads and publishes to feedback //// Curently sending arm angles instead
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


    // Takes 8 Misc_Angles and writes to servos ID 12-21
    void Misc_AnglesCallback(const std_msgs::Float32MultiArray& Misc_Angles){
        // Clears bulk write stack
        groupBulkWrite.clearParam();

        // Creates and assigns array with each byte of message
        uint8_t data_array[4];
        for (int i = 1; i < 9; i++){     // Do for all 8ish misc servos
            if(Misc_Angles.data[i-1] != -1){
                uint32_t data = (unsigned int)(Misc_Angles.data[i - 1]); // Convert int32 to uint32
                data_array[0] = DXL_LOBYTE(DXL_LOWORD(data));
                data_array[1] = DXL_HIBYTE(DXL_LOWORD(data));
                data_array[2] = DXL_LOBYTE(DXL_HIWORD(data));
                data_array[3] = DXL_HIBYTE(DXL_HIWORD(data));
                groupBulkWrite.addParam((i + 11), GOAL_POSITION_ADDR, 4, data_array);  // Adds message to stack arguments(servo ID, Address, size, data array of bytes)
            }
        }
        groupBulkWrite.txPacket();  // Write servos with prepared list all at once

        // Sets the misc moving flag to start checking for servos to complete move
        misc_moving = 1;

        // Print final servo values to ros
        //ROS_INFO("Angles: %f, %f, %f, %f, %f, %f, %f, %f", Misc_Angles.data[0], Misc_Angles.data[1], Misc_Angles.data[2], Misc_Angles.data[3], Misc_Angles.data[4], Misc_Angles.data[5], Misc_Angles.data[6], Misc_Angles.data[7]);
    }


    // Checks if servos are within exceptable error to declare done
    void miscMoving(){
        // Clears bulk read stack
        groupBulkRead.clearParam();

        // Reads goal angles of misc servos and stores in array
        for (int i = 12; i < 12 + MISC_COUNT; i++){
            groupBulkRead.addParam(i, 116, 4);
        }
        groupBulkRead.txRxPacket(); // Executes bulk read

        // Assigns temp bulk read array to topic and publishes
        float desired_angle[8];
        for (int i = 0; i <  MISC_COUNT; i++){
            desired_angle[i] = groupBulkRead.getData((i + 12), 116, 4);
        }
        // Clears bulk read stack
        groupBulkRead.clearParam();

        // Reads servo posistions of servos and stores in array
        for (int i = 12; i < 12 + MISC_COUNT; i++){
            groupBulkRead.addParam(i, 132, 4);
        }
        groupBulkRead.txRxPacket(); // Executes bulk read

        // Checks if current posistion and goal posistion are withen acceptable toleracne
        for (int i = 0; i < MISC_COUNT; i++){
            if(abs(desired_angle[i] - groupBulkRead.getData((i + 12), 132, 4)) > MISC_ANGLE_TOLERANCE){ // Used to use uint32_t pos[8]; for storing
                misc_moving = 1;                 // Joint error is to large que up another check
                return;                         //  break out of function
            }
        }
        misc_moving = 0;                 // All joints in acceptable range so remove funtion for loop
        Misc_Done.data = 1;              //  mark servos as done moving
        Misc_Done_pub.publish(Misc_Done); //  publish done moving result
    }


    // Accepts move instructions for bot x, y, z, speed and sets local values to trigger on sensor callbacks
    void MoveCallback(const std_msgs::Float32MultiArray& Move){
        // Copy x&y offset, desired z bearing, and max speed allowed to locals for callback pids
        desired_x = Move.data[0];
        desired_y = Move.data[1];
        desired_z = Move.data[2];
        max_speed = Move.data[3] * MAX_SPEED;

        // Reset arrived tally to not trigger imediatly
        arrived_x = 0;
        arrived_y = 0;
        arrived_z = 0;

        // Reset backup saftey
        e_stop = 0;
    }


    // Triggers on /TOF_Front post sets linear y speed based on pids
    void TOF_FrontCallback(const std_msgs::Int16& TOF_Front){
        // If desired_y = 0 stop y movement, ingore y sensor, declare arrived
        if(desired_y == 0){
            linear_y = 0;
            arrived_y = SEQUENTIAL_READS;
        }

        // If desired y is negative move backwards at that speed
        else if(desired_y < 0){
            linear_y = -1000 * desired_y;
        }


        // If desired y is 1 move forward at max speed
        else if(desired_y == 1){
            linear_y = -1000 * desired_y;
        }

        // If desired y is positive use sensor data to calculate pid for y velocity
        else{
            double error_y = desired_y - TOF_Front.data;
            error_y_cumulative += error_y;

            // Constrain error to plus or minus CUMLATIVE_CAP
            if(error_y_cumulative > CUMULATIVE_CAP)
                error_y_cumulative = CUMULATIVE_CAP;
            if(error_y_cumulative < -CUMULATIVE_CAP)    
                error_y_cumulative = -CUMULATIVE_CAP;

            // Calculate pid error based on K's and error     
            linear_y = (KP_Y * error_y) + (KI_Y * error_y_cumulative / TS) + (KD_Y * TS * (error_y - error_y_prev));
            error_y_prev = error_y;

            // If we are close enough to desired value tally up how long we are here
            if(abs(error_y) <= ALLOWABLE_ERROR){
                if(arrived_y < SEQUENTIAL_READS)    // prevent overflow in counting
                    arrived_y++;
            }
            else// If we have moved to far or overshoot, reset tally
                arrived_y = 0;

        }
        // Call kinematics with updated linear_y value
        botKinematics();
    }


    // Triggers on /TOF_Left post sets linear x speed based on pids
    void TOF_LeftCallback(const std_msgs::Int16& TOF_Left){
        // If desired_x = 0 stop x movement, ingore x sensor, declare arrived
        if(desired_x == 0){
            linear_x = 0;
            arrived_x = SEQUENTIAL_READS;
        } 

        // If desired x is negative use sensor data to calculate pid for x velocity from left sensor
        else if(desired_x < 0){
            double error_x = -desired_x - TOF_Left.data;
            error_x_cumulative += error_x;

            // Constrain error to plus or minus CUMLATIVE_CAP
            if(error_x_cumulative > CUMULATIVE_CAP)
                error_x_cumulative = CUMULATIVE_CAP;
            if(error_x_cumulative < -CUMULATIVE_CAP)    
                error_x_cumulative = -CUMULATIVE_CAP;

            // Calculate pid error based on K's and error 
            linear_x = (KP_X * error_x) + (KI_X * error_x_cumulative / TS) + (KD_X * TS * (error_x - error_x_prev));
            error_x_prev = error_x;

            // If we are close enough to desired value tally up how long we are here
            if(abs(error_x) <= ALLOWABLE_ERROR){
                if(arrived_x < SEQUENTIAL_READS)    // prevent overflow in counting
                    arrived_x++;
            }
            else// If we have moved to far or overshoot, reset tally
                arrived_x = 0;
        }
        // Call kinematics with updated linear_x value
        botKinematics();
    }


    // Triggers on /TOF_Right post sets linear x speed based on pids
    void TOF_RightCallback(const std_msgs::Int16& TOF_Right){
        // If desired_x = 0 stop x movement, ingore x sensor, declare arrived
        if(desired_x == 0){
            linear_x = 0;
            arrived_x = SEQUENTIAL_READS;
        }

        // If desired x is positive use sensor data to calculate pid for x velocity from right sensor
        else if(desired_x > 0){
            double error_x = -1 * (desired_x - TOF_Right.data);
            error_x_cumulative += error_x;

            // Constrain error to plus or minus CUMLATIVE_CAP
            if(error_x_cumulative > CUMULATIVE_CAP)
                error_x_cumulative = CUMULATIVE_CAP;
            if(error_x_cumulative < -CUMULATIVE_CAP)    
                error_x_cumulative = -CUMULATIVE_CAP;

            // Calculate pid error based on K's and error
            linear_x = (KP_X * error_x) + (KI_X * error_x_cumulative / TS) + (KD_X * TS * (error_x - error_x_prev));
            error_x_prev = error_x;

            // If we are close enough to desired value tally up how long we are here
            if(abs(error_x) <= ALLOWABLE_ERROR){
                if(arrived_x < SEQUENTIAL_READS)    // prevent overflow in counting
                    arrived_x++;
            }
            else// If we have moved to far or overshoot, reset tally
                arrived_x = 0;
        }
        // Call kinematics with updated linear_x value
        botKinematics();
    }


    // Triggers on /IMU_Bearing post sets rotation z spead based on pids and offset
    void IMU_BearingCallback(const std_msgs::Int16& IMU_Bearing){
        // If bearing_offset is negative it is uninitialized, so set it as offset
        if(bearing_offset == -1 && desired_z != -1)
            bearing_offset = IMU_Bearing.data;
        
        // Find difference or errot between desired and actual
        double error_z = desired_z - (IMU_Bearing.data - bearing_offset);

        // Constrain error to plus or minus 180 deg
        if(error_z > 180)
            error_z -= 360;
        if(error_z < -180)
            error_z += 360;

        // Calculate pid error based on K's and error
        error_z_cumulative += error_z;

        // Constrain error to plus or minus CUMLATIVE_CAP
        if(error_z_cumulative > CUMULATIVE_CAP)
            error_z_cumulative = CUMULATIVE_CAP;
        if(error_z_cumulative < -CUMULATIVE_CAP)    
            error_z_cumulative = -CUMULATIVE_CAP;

        // Calculate pid error based on K's and error
        linear_z = (KP_Z * error_z) + (KI_Z * error_z_cumulative / TS) + (KD_Z * TS * (error_z - error_z_prev));
        error_z_prev = error_z;

        // If we are close enough to desired value tally up how long we are here
        if(abs(error_z) <= ALLOWABLE_ERROR){
            if(arrived_z < SEQUENTIAL_READS)    // prevent overflow in counting
                arrived_z++;
        }
        else// If we have moved to far or overshoot, reset tally
            arrived_z = 0;

        // Call kinematics with updated linear_z value
        botKinematics();
    }


    // Backup safety
    void TOF_BackCallback(const std_msgs::Int16& TOF_Back){
        if(TOF_Back.data > 120 && desired_y < 0)
            e_stop = 1;
    }


    // Takes xy velocity and z rotation and calculates wheel speeds scaled to max speed
    void botKinematics(){
        // Find angle of linear movement for kinematics
        double theta = atan2(linear_y, linear_x);
        // Make angle positive
        if(theta < 0)
            theta += 6.28;

        // Scale speed based on x, y, and max speeds
        double speed = sqrt(linear_x * linear_x + linear_y * linear_y);
        if(speed > max_speed)
            speed = max_speed;

        // Inverse wheel kinematics
        Wheel_Speeds.data[0] = speed * cos(theta) + linear_z;
        Wheel_Speeds.data[1] = speed * cos(theta + (2.0 / 3.0 * 3.1415)) + linear_z;
        Wheel_Speeds.data[2] = speed * cos(theta - (2.0 / 3.0 * 3.1415)) + linear_z;
        
        // Find max value of wheels
        double max = abs(Wheel_Speeds.data[0]);
        if(abs(Wheel_Speeds.data[1]) > max)
            max = abs(Wheel_Speeds.data[1]); 
        if(abs(Wheel_Speeds.data[2]) > max) 
            max = abs(Wheel_Speeds.data[2]); 

        // If max value greater than fastest wheel speeds scale all speeds down
        if(max > 255){
            Wheel_Speeds.data[0] = Wheel_Speeds.data[0] / max * 255;
            Wheel_Speeds.data[1] = Wheel_Speeds.data[1] / max * 255;
            Wheel_Speeds.data[2] = Wheel_Speeds.data[2] / max * 255;
        }

        //ROS_INFO("Arrived array: %f, %f, %f", arrived_x, arrived_y, arrived_z);

        // Checks to see if xyz are all stable and happy and we arn't about to die
        if(e_stop == 1 || (max_speed != 0 && arrived_x >= SEQUENTIAL_READS && arrived_y >= SEQUENTIAL_READS && arrived_z >= SEQUENTIAL_READS)){
            // Ignore sensors and set speed to 0
            //desired_x = 0;
            //desired_y = 0;
            //desired_z = 0;
            max_speed = 0;

            // Publish move done
            Move_Done.data = 1;
            Move_Done_pub.publish(Move_Done);
        }

        // If max_Speed 0 overide all orders with stop
        if(max_speed == 0){
            Wheel_Speeds.data[0] = 0;
            Wheel_Speeds.data[1] = 0;
            Wheel_Speeds.data[2] = 0;
        }

        // Publish speed to wheels
        Wheel_Speeds_pub.publish(Wheel_Speeds); 
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
 

// Pub, sub, and ros declarations
private: 
    ros::NodeHandle nh;

    // Publisher ros declarations
    ros::Publisher Feedback_pub;
    ros::Publisher Arm_Angles_pub;
    ros::Publisher Arm_Done_pub;
    ros::Publisher Move_Done_pub;
    ros::Publisher Wheel_Speeds_pub;
    ros::Publisher Misc_Done_pub;
    ros::Publisher Task_Space_pub;

    // Subscriber ros declarations
    ros::Subscriber Get_Feedback_sub;
    ros::Subscriber Task_Space_sub;
    ros::Subscriber Misc_Angles_sub;
    ros::Subscriber Wheel_Speeds_sub;
    ros::Subscriber Arm_Angles_sub;
    ros::Subscriber Move_sub;
    ros::Subscriber TOF_Front_sub;
    ros::Subscriber TOF_Left_sub;
    ros::Subscriber TOF_Right_sub;
    ros::SUbscriber TOF_Back_sub;
    ros::Subscriber IMU_Bearing_sub;
    ros::Subscriber PIDs_sub;

    // Publisher variable declarations
    std_msgs::Float32MultiArray Feedback;
    std_msgs::Float32MultiArray Arm_Angles;
    std_msgs::Float32MultiArray Task_Space;

    // Local publisher variables
    std_msgs::Float32MultiArray Wheel_Speeds;
    std_msgs::Int8 Arm_Done;
    std_msgs::Int8 Move_Done;
    std_msgs::Int8 Misc_Done;

    // Variables for bot movement functions
    double  desired_x = 0, error_x_prev = 0, error_x_cumulative = 0, linear_x = 0, arrived_x = 0,
            desired_y = 0, error_y_prev = 0, error_y_cumulative = 0, linear_y = 0, arrived_y = 0,
            desired_z = -1, error_z_prev = 0, error_z_cumulative = 0, linear_z = 0, arrived_z = 0,
            max_speed = 0, bearing_offset = -1, e_stop = 0;

    // Local task space 
    float local_task_space[6];
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
        if(ServoObject.arm_moving)
            ServoObject.armMoving();
        
        // Once triggered by new miscCallback will keep running until close enough to desired posistion
        if(ServoObject.misc_moving)
            ServoObject.miscMoving();
        

        ros::spinOnce();
        //spinner.spin();
    }

    // Close ports and stop node
    packetHandler->write4ByteTxOnly(portHandler, ARM_SECONDARY_ID, GOAL_VELOCITY_ADDR, STOP);               // Sets velocity to stop
    portHandler->closePort();
    return 0;
}

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
#include <time.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

using namespace dynamixel;

// Address table of servos
#define TORQUE_ENABLE_ADDR      64
#define LED_ADDR                65
#define GOAL_POSITION_ADDR      116
#define PRESENT_LOAD_ADDR       126
#define MAX_VEL_ADDR            112
#define MAX_ACC_ADDR            108
#define PRESENT_POSITION_ADDR   132
#define GOAL_VELOCITY_ADDR      104
#define POS_P_GAIN_ADDR         84
#define POS_I_GAIN_ADDR         82
#define POS_D_GAIN_ADDR         80
#define GOAL_CURRENT_ADDR       102
#define NUM_BYTES_1             1
#define NUM_BYTES_2             2
#define NUM_BYTES_4             4

// Defined servo and arm values
#define SECONDARY_ID            100     // Secondary ID for writing multiple servos at once
#define MAX_VEL                 1000    //100    // Max velocity of arm joints
#define MAX_ACC                 200     //2      // Max acceleration of arm joints
#define TORQUE_ENABLE           1       // Enable torque on servos
#define CENTER_POSISTION        2048    // Center posistion for angle mode on servos
#define POS_P_GAIN              640     // Angle mode P gain
#define POS_I_GAIN              500     // Angle mode I gain
#define POS_D_GAIN              4000    // Angle mode D gain
#define STOP                    0       // Velocity mode stop
#define MISC_ANGLE_TOLERANCE    50//10      // Counts we need to be off to count as arrived
//#define ARM_TOLERANCE           10      // Allowable error in arm before declareing arrived
#define STEPS_DOWN              10      // Number of millimeters to move down per step
#define MAX_CURRENT             500     // Max current small servos can pull
#define DOWN_SPEED_SCALE        1       // Scale of acc's/vel's for linear interpolation
#define DOWN_WAIT               150     // Milli seconds to wait before moving down for linear interpolation
#define MAX_DOWN_SPEED          50      // Max acc's/vel's for lineal interpolation
#define RESTART_TIME            10      // Seconds between check/restart torqued out servos
#define ARDUINO_TIMEOUT         500     // Millisecond safety between arduino signal that pauses wheel movement
#define MAX_LOAD                100     // Load threshold needed to assume servo has arrived range 0-1000 = 0-100%

// #if macros
#define DEBUG                   1       // Prints out ros info's
#define CUSTOM_PIDS             0       // Set to 1 for custom variable pids, or 0 for preset

// PID's and Move constants x = left/right, y = forward/backwards, z = rotation
#if CUSTOM_PIDS// Custom PIDs for use with publishing
double KP_X = 6.0, KI_X = 0.01, KD_X = 0.3, KP_Y = 6.0, KI_Y = 0.01, KD_Y = 0.3, KP_Z = 20.0, KI_Z = 0.01, KD_Z = 0.4, ALLOWABLE_ERROR = 5, SEQUENTIAL_READS = 5;
//double KP_X = 4.0, KI_X = 0.01, KD_X = 0.5, KP_Y = 4.0, KI_Y = 0.01, KD_Y = 0.5, KP_Z = 6.0, KI_Z = 0.01, KD_Z = 0.3, ALLOWABLE_ERROR = 5, SEQUENTIAL_READS = 5; // old wheels

#else // Preset PIDs
#define KP_X    6.0//7.0
#define KI_X    0.01//0.1
#define KD_X    0.3//0.8

#define KP_Y    6.0//7.0
#define KI_Y    0.01//0.1
#define KD_Y    0.3//0.8

#define KP_Z    20.0//10.0
#define KI_Z    0.01//0.01
#define KD_Z    0.4//0.5

#define ALLOWABLE_ERROR     5//5  // How close we need to be to the final posistion for each axis in mm
#define SEQUENTIAL_READS    5//5  // How many readings we need to be in these posistion before declaring arrival about 10 reading/sec
#endif

// Other PID and wheel constants
#define MAX_SPEED           460//255        // Maximum velocity of wheeled servos
#define TS                  10      // Estimation of TOF samples per sec
#define MAX_SPEED_SCALE     5       // Scaler for max_speed of wheels
#define CUMULATIVE_CAP      1000    // Cap on cumlative error for pid
#define LIFTUP_SAFETY       200     // Distance bot must be picked up to stop
#define BACKUP_SAFETY       120     // Distance that will halt a backup move if read from back tof

// Default dynamixel setting
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT89FKZ2-if00-port0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Set up dynamixel ports and packets
PortHandler* portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler* packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Creates objects for sync reading and writing of servos
GroupSyncRead groupSyncRead_armPresPos(portHandler, packetHandler, PRESENT_POSITION_ADDR, NUM_BYTES_4);
GroupSyncRead groupSyncRead_miscGoalPos(portHandler, packetHandler, GOAL_POSITION_ADDR, NUM_BYTES_4);
GroupSyncRead groupSyncRead_miscPresPos(portHandler, packetHandler, PRESENT_POSITION_ADDR, NUM_BYTES_4);
GroupSyncRead groupSyncRead_torque(portHandler, packetHandler, TORQUE_ENABLE_ADDR, NUM_BYTES_1);
GroupSyncRead groupSyncRead_miscPresLoad(portHandler, packetHandler, PRESENT_LOAD_ADDR, NUM_BYTES_2);

GroupSyncWrite groupSyncWrite_armGoalPos(portHandler, packetHandler, GOAL_POSITION_ADDR, NUM_BYTES_4);
GroupSyncWrite groupSyncWrite_armAcc(portHandler, packetHandler, MAX_ACC_ADDR, NUM_BYTES_4);
GroupSyncWrite groupSyncWrite_armVel(portHandler, packetHandler, MAX_VEL_ADDR, NUM_BYTES_4);
GroupSyncWrite groupSyncWrite_miscGoalPos(portHandler, packetHandler, GOAL_POSITION_ADDR, NUM_BYTES_4);
GroupSyncWrite groupSyncWrite_wheelGoalVel(portHandler, packetHandler, GOAL_VELOCITY_ADDR, NUM_BYTES_4);


// Main class holding all servo declarations, functions, and variables
class ServoClass{
public:
    // Int flags for main
    int arm_moving = 0, misc_moving = 0, sync_arm_goal_pos = 0, sync_arm_acc_vel = 0, 
        sync_misc_goal_pos = 0, sync_wheel_goal_vel = 0;

    // Clock timers
    clock_t restart_check_time = 10000000, arduino_timeout_time;

    // Initiating pubs, subs, and arrays
    ServoClass(ros::NodeHandle* nodehandle){ 
        nh = *nodehandle;       

        // Resize Publisher Arrays
        Arm_Angles.data.resize(10);
        Feedback.data.resize(8);
        Task_Space.data.resize(7);

        // Publishers
        Feedback_pub = nh.advertise<std_msgs::Float32MultiArray>("/Feedback", 1);
        Arm_Angles_pub = nh.advertise<std_msgs::Float32MultiArray>("/Arm_Angles", 1);
        Arm_Done_pub = nh.advertise<std_msgs::Int8>("/Arm_Done", 1);
        Move_Done_pub = nh.advertise<std_msgs::Int8>("/Move_Done", 1);
        Misc_Done_pub = nh.advertise<std_msgs::Int8>("/Misc_Done", 1);
        Task_Space_pub = nh.advertise<std_msgs::Float32MultiArray>("/Task_Space", 1);
        Local_En_pub = nh.advertise<std_msgs::Bool>("/Local_En", 1);

        // Subscribers
        Get_Feedback_sub = nh.subscribe("/Get_Feedback", 1, &ServoClass::Get_FeedbackCallback, this);
        Task_Space_sub = nh.subscribe("/Task_Space", 1, &ServoClass::Task_SpaceCallback, this);
        Misc_Angles_sub = nh.subscribe("/Misc_Angles", 1, &ServoClass::Misc_AnglesCallback, this);
        Arm_Angles_sub = nh.subscribe("/Arm_Angles", 1, &ServoClass::Arm_AnglesCallback, this);
        Move_sub = nh.subscribe("/Move", 1, &ServoClass::MoveCallback, this);
        PIDs_sub = nh.subscribe("/PIDs", 1, &ServoClass::PIDsCallback, this);
        Local_Data_sub = nh.subscribe("/Local_Data", 1, &ServoClass::Local_DataCallback, this);
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
    void initializeServos() {
        // Starts communications with servos
        portHandler->openPort();            // Opens port to U2D2
        portHandler->setBaudRate(BAUDRATE); // Sets default baud rate

        // Sets up initial values and states for servos
        packetHandler->write1ByteTxOnly(portHandler, SECONDARY_ID, TORQUE_ENABLE_ADDR, TORQUE_ENABLE);      // Enable torque
        usleep(10000);
        packetHandler->write1ByteTxOnly(portHandler, SECONDARY_ID, LED_ADDR, 1);      // Turn on LED
        usleep(10000);
        packetHandler->write2ByteTxOnly(portHandler, 8, GOAL_CURRENT_ADDR, MAX_CURRENT);         // Sets gripper servo current limit
        
        // Sets up sync read params for all torques
        for(int i = 1; i < 16; i++)
            groupSyncRead_torque.addParam(i);
        
        // Sets up sync read params for arm present pos
        for(int i = 1; i < 9; i++)
            groupSyncRead_armPresPos.addParam(i);

        // Sets up sync read params for misc goal pos, present pos, and present load
        for(int i = 12; i < 16; i++){
            groupSyncRead_miscGoalPos.addParam(i);
            groupSyncRead_miscPresPos.addParam(i);
            groupSyncRead_miscPresLoad.addParam(i);
        }
            
        // Publish 8 starting arm angles and speed to Arm_Angles
        int Arm_Start_Angles[10] = { 2009, 2721, 2706, 1635, 2166, 1661, 1886, 1980, 10, 500 };
        for (int i = 0; i < 10; i++) {
            Arm_Angles.data[i] = Arm_Start_Angles[i];
        }
        Arm_Angles_pub.publish(Arm_Angles);


        // Create and write misc servos to custom starting pos
        int Misc_Start_Angles[4] = { 2048, 1050, 1400, 2048 };  //3) 1050 -> 1400

        // Creates and assigns array with each byte of message
        uint8_t data_array[4];
        for (int i = 1; i < 5; i++){     // Do for all 4ish misc servos
            if(Misc_Start_Angles[i-1] != -1){
                uint32_t data = (unsigned int)(Misc_Start_Angles[i - 1]); // Convert int32 to uint32
                data_array[0] = DXL_LOBYTE(DXL_LOWORD(data));
                data_array[1] = DXL_HIBYTE(DXL_LOWORD(data));
                data_array[2] = DXL_LOBYTE(DXL_HIWORD(data));
                data_array[3] = DXL_HIBYTE(DXL_HIWORD(data));
                groupSyncWrite_miscGoalPos.addParam((i + 11), data_array);  // Adds message to stack arguments(servo ID, data array of bytes)
            }
        }

        // Sets flag to sync write
        sync_misc_goal_pos = 1;

        // Delay to wait for servos connected
        usleep(100000);
        #if DEBUG
            ROS_WARN("**********Node initialized");
        #endif
    }


    // Takes global x, y, z, theta(wrist), phi(claw), speed, and tolerance and pubs servo values to Arm_Angles
    void Task_SpaceCallback(const std_msgs::Float32MultiArray& Task_Space){
        // Optional debug info
        #if DEBUG
            ROS_WARN("**********Taskspace recieved: %f, %f, %f, %f, %f, %f, %f", Task_Space.data[0], Task_Space.data[1], Task_Space.data[2], Task_Space.data[3], Task_Space.data[4], Task_Space.data[5], Task_Space.data[6]);
        #endif

        // Declares variables and copies taskspace from topic
        float x, y, z, theta, phi;
        x = Task_Space.data[0];
        y = Task_Space.data[1];
        z = Task_Space.data[2];
        theta = Task_Space.data[3];
        phi = Task_Space.data[4];

        // If speed was negative then move down at STEPS_DOWN pace
        if(Task_Space.data[5] < 0){
            // Declare moving down
            moving_down = 1;

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
                local_task_space[5] = 10;
            }

            // Update y value
            local_task_space[1] = y;
        }
        else // No longer moving down slowly
            moving_down = 0;

        //Print x, y, z recieved to ros
        //ROS_INFO("%d, %d, %d, %d, %d]", (int)x, (int)y, (int)z, (int)theta, (int)phi);

        // Creat arrays for calculations and predefined values
        float q[6] = { 0, 0, 0, 0, 0, 0 };                          // Major joint angles of robot
        int Q[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };                      // Final joint angles for servos including duplicated j2 and claw
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
        Q[9] = Task_Space.data[6];

        // Publish 8 servo angles and speed to Arm_Angles
        for (int i = 0; i < 10; i++){
            Arm_Angles.data[i] = Q[i];
        }
        Arm_Angles_pub.publish(Arm_Angles);
    }


    // Takes 8 Arm_Angles, speed, and tolerance and writes to servos ID 1-8
    void Arm_AnglesCallback(const std_msgs::Float32MultiArray& Arm_Angles){
        // Optional debug info
        #if DEBUG
            ROS_WARN("**********Arm Angles recieved: %f, %f, %f, %f, %f, %f, %f, %f", Arm_Angles.data[0], Arm_Angles.data[1], Arm_Angles.data[2], Arm_Angles.data[3], Arm_Angles.data[4], Arm_Angles.data[5], Arm_Angles.data[6], Arm_Angles.data[7]);
        #endif

        // Create array for arm speeds defaulting to received speeds
        double speed[8] = {Arm_Angles.data[8], Arm_Angles.data[8], Arm_Angles.data[8], Arm_Angles.data[8], Arm_Angles.data[8], Arm_Angles.data[8], Arm_Angles.data[8], Arm_Angles.data[8]};

        // If moving down using linear interp scales speeds
        if(moving_down){
            for(int i = 0; i < 8; i++){
                double delta_arm_angles = abs(Arm_Angles.data[i] - last_arm_angles[i]);     // Calculate change in each joint
                speed[i] = delta_arm_angles * DOWN_SPEED_SCALE;                             // Scale each joint change to get acc's/vel's
                if(delta_arm_angles == 0)                                                   // If arm speed of 0 set to minimum
                    speed[i] = 1;
                if(speed[i] > MAX_DOWN_SPEED)                                               // Prevent crazy speeds
                    speed[i] = MAX_DOWN_SPEED;
            }

            // Start timing from move down issued
            next_down_time = clock() + DOWN_WAIT * 1000;
        }

        // Echo jont speeds
        //ROS_WARN("**********Speeds: %f, %f, %f, %f, %f, %f, %f, %f", speed[0], speed[1], speed[2], speed[3], speed[4], speed[5], speed[6], speed[7]);

        // Store new angles in last angles for next time
        for(int i = 0; i < 8; i++)
            last_arm_angles[i] = Arm_Angles.data[i];

        // Write accel and speed for servos of arm
        armSpeed(speed);

        // Sets arm tolerance for when to delcare done
        arm_tolerance = Arm_Angles.data[9];

        // Creates and assigns array with each byte of message
        uint8_t data_array[4];
        for (int i = 1; i < 9; i++){    // Do for all 8 arm servos
            uint32_t data = (unsigned int)(Arm_Angles.data[i - 1]); // Convert int32 to uint32
            data_array[0] = DXL_LOBYTE(DXL_LOWORD(data));
            data_array[1] = DXL_HIBYTE(DXL_LOWORD(data));
            data_array[2] = DXL_LOBYTE(DXL_HIWORD(data));
            data_array[3] = DXL_HIBYTE(DXL_HIWORD(data));
            groupSyncWrite_armGoalPos.addParam(i, data_array);  // Adds message to stack arguments(servo ID, data array of bytes)
        }

        sync_arm_goal_pos = 1; 

        // Sets the arm moving flag to start checking for arm to complete move
        arm_moving = 1;
    }


    // Checks if robot arm is within exceptable error to declare it has arrived
    void armMoving(){
        // If moving down for linear interp just time, dont check arrival
        if(moving_down && local_task_space[5] < 0){
            // If enough time has elapsed to next step
            if(next_down_time < clock()){
                // Copy local variable and pub
                for(int i = 0; i < 6; i++)
                    Task_Space.data[i] = local_task_space[i];
                Task_Space_pub.publish(Task_Space);
            }
        }
        else{ // Not moving down in steps
            // Executes sync read
            groupSyncRead_armPresPos.txRxPacket(); 

            // Checks if current posistion and goal posistion are withen acceptable toleracne
            for (int i = 0; i < 8; i++){
                if(abs(Arm_Angles.data[i] - groupSyncRead_armPresPos.getData((i + 1), PRESENT_POSITION_ADDR, 4)) > arm_tolerance){ // Used to use uint32_t pos[8]; for storing
                    arm_moving = 1;                 // Joint error is to large que up another check
                    //Arm_Done.data = 0;              //  mark arm as not done moving
                    //Arm_Done_pub.publish(Arm_Done); //  publish not done moving result
                    return;                         //  break out of function
                }
            }

            // ROS INFO
            #if DEBUG
                ROS_WARN("**********Arm arrived");
            #endif

            arm_moving = 0;                 // All joints in acceptable range so remove funtion for loop
            Arm_Done.data = 1;              //  mark arm as done moving
            Arm_Done_pub.publish(Arm_Done); //  publish done moving result
        }
    }


    // Writes accels and velocity to the servos
    void armSpeed(double* speed){

        // Calculate accs/vels and add to stack to write
        for (int i = 1; i < 9; i++){     // Do for all 8 servos
            // Scales speed of vel and acc
            uint32_t max_acc = (uint32_t)(MAX_ACC / 100 - 2 + speed[i - 1] * 0.4); // 2-40
            uint32_t max_vel = (uint32_t)(MAX_VEL / 100 - 10 + speed[i - 1] * 4); // 10-400

            // Creates and assigns array with each byte of messages
            uint8_t data_array_acc[4] = {DXL_LOBYTE(DXL_LOWORD(max_acc)), DXL_HIBYTE(DXL_LOWORD(max_acc)), DXL_LOBYTE(DXL_HIWORD(max_acc)), DXL_HIBYTE(DXL_HIWORD(max_acc))};
            uint8_t data_array_vel[4] = {DXL_LOBYTE(DXL_LOWORD(max_vel)), DXL_HIBYTE(DXL_LOWORD(max_vel)), DXL_LOBYTE(DXL_HIWORD(max_vel)), DXL_HIBYTE(DXL_HIWORD(max_vel))};

            // Add accs/vels to sync stack
            groupSyncWrite_armAcc.addParam((i), data_array_acc);  // Adds message to stack arguments(servo ID, data array of bytes)
            groupSyncWrite_armVel.addParam((i), data_array_vel);  // Adds message to stack arguments(servo ID, data array of bytes)
        }

        // Set flag to sync write once outside of callback
        sync_arm_acc_vel = 1;
    }


    // Read present posistion and publishes to feedback
    void Get_FeedbackCallback(const std_msgs::Int8& Get_Feedback){
        // Executes sync read
        groupSyncRead_armPresPos.txRxPacket();

        // Assigns temp sync read array to topic and publishes
        for (int i = 0; i < 8; i++){
            Feedback.data[i] = groupSyncRead_armPresPos.getData((i + 1), 132, 4);
        }
        Feedback_pub.publish(Feedback); // Publishes 8x array of arm servo posistions
    }


    // Reads and restarts servos that have torqued out
    void readRestart(){
        // Set timer for next check
        restart_check_time = clock() + RESTART_TIME * 1000000;
        
        // Executes sync read
        groupSyncRead_torque.txRxPacket();

        // Variable to hold error(not used)
        uint8_t dxl_error = 0;

        // Reboots any servo that doesn't have torque enabled
        for (int i = 1; i < 16; i++){
            if(!groupSyncRead_torque.getData((i), TORQUE_ENABLE_ADDR, NUM_BYTES_1))
                packetHandler->reboot(portHandler, i, &dxl_error);
        }

        // Sets gripper servo current limit
        packetHandler->write2ByteTxOnly(portHandler, 8, GOAL_CURRENT_ADDR, MAX_CURRENT);
    }


    // Takes 4 Misc_Angles and writes to servos ID 12-15
    void Misc_AnglesCallback(const std_msgs::Float32MultiArray& Misc_Angles){
        // Optional debug info
        #if DEBUG
            ROS_WARN("**********Misc Angles recieved: %f, %f, %f, %f", Misc_Angles.data[0], Misc_Angles.data[1], Misc_Angles.data[2], Misc_Angles.data[3]);
        #endif
        
        // Creates and assigns array with each byte of message
        uint8_t data_array[4];
        for (int i = 1; i < 5; i++){     // Do for all 4 misc servos
            if(Misc_Angles.data[i-1] >= 0){
                uint32_t data = (unsigned int)(Misc_Angles.data[i - 1]); // Convert int32 to uint32
                data_array[0] = DXL_LOBYTE(DXL_LOWORD(data));
                data_array[1] = DXL_HIBYTE(DXL_LOWORD(data));
                data_array[2] = DXL_LOBYTE(DXL_HIWORD(data));
                data_array[3] = DXL_HIBYTE(DXL_HIWORD(data));
                groupSyncWrite_miscGoalPos.addParam((i + 11), data_array);  // Adds message to stack arguments(servo ID, data array of bytes)
            }
        }

        if(Misc_Angles.data[2] == -2)
            packetHandler->write1ByteTxOnly(portHandler, 14, TORQUE_ENABLE_ADDR, TORQUE_ENABLE);      // Enable torque for servo 14
        else if(Misc_Angles.data[2] == -3)
            packetHandler->write1ByteTxOnly(portHandler, 14, TORQUE_ENABLE_ADDR, 0);      // Disable torque for servo 14
        else if(Misc_Angles.data[2] == -4){
            uint8_t dxl_error = 0;
            packetHandler->reboot(portHandler, 14, &dxl_error);
            //usleep(100000);
            //packetHandler->write1ByteTxOnly(portHandler, 14, TORQUE_ENABLE_ADDR, 1);      // Disable torque for servo 14
            //usleep(10000);
            //packetHandler->write1ByteTxOnly(portHandler, 14, LED_ADDR, 1);      // Disable torque for servo 14
        }
        
        // Sets flag to sync write outside of callback
        sync_misc_goal_pos = 1;
    }


    // Checks if servos are within exceptable error to declare done
    void miscMoving(){
        // Executes sync read of goal posistion
        groupSyncRead_miscGoalPos.txRxPacket();

        // Executes sync read of present posistion
        groupSyncRead_miscPresPos.txRxPacket();

        // Executes sync read of present loads
        groupSyncRead_miscPresLoad.txRxPacket();

        // Checks if current posistion and goal posistion are withen acceptable toleracne
        for (int i = 12; i < 16; i++){
            if(abs((int32_t)(groupSyncRead_miscGoalPos.getData((i), GOAL_POSITION_ADDR, NUM_BYTES_4)) - (int32_t)(groupSyncRead_miscPresPos.getData((i), PRESENT_POSITION_ADDR, NUM_BYTES_4))) > MISC_ANGLE_TOLERANCE){ // Used to use uint32_t pos[8]; for storing
                // If this servo is under heavy load, assume it has arrived and skip return
                if(groupSyncRead_miscPresLoad.getData((i), PRESENT_LOAD_ADDR, NUM_BYTES_2) < MAX_LOAD)
                    return; //  break out of function signifing servos still moving
            }
        }
        // Ros Info
        #if DEBUG
            ROS_WARN("**********Misc arrived");
        #endif
        misc_moving = 0;                 // All joints in acceptable range so remove funtion for loop
        Misc_Done.data = 1;              //  mark servos as done moving
        Misc_Done_pub.publish(Misc_Done); //  publish done moving result
    }


    // Accepts move instructions for bot x, y, z, speed and sets local values to trigger on sensor callbacks
    void MoveCallback(const std_msgs::Float32MultiArray& Move){
        // Optional debug info
        #if DEBUG
            ROS_WARN("**********Move command received: %f, %f, %f, %f", Move.data[0], Move.data[1], Move.data[2], Move.data[3]);
        #endif

        // Copy x&y offset, desired z bearing, and max speed allowed to locals for callback pids
        desired_x = Move.data[0];
        desired_y = Move.data[1];
        desired_z = Move.data[2];
        max_speed = Move.data[3] * MAX_SPEED_SCALE;

        // Reset arrived tally to not trigger imediatly
        arrived_x = 0;
        arrived_y = 0;
        arrived_z = 0;

        // Reset backup saftey
        e_stop = 0;

        // Reset Cumulative I values
        error_x_cumulative = 0;
        error_y_cumulative = 0;
        error_z_cumulative = 0;

        //
        Local_En.data = true;
        Local_En_pub.publish(Local_En);
    }


    // Accepts sensor data and calls individual funtions to calculate pids
    void Local_DataCallback(const std_msgs::Int16MultiArray& Local_Data){
        tofFront(Local_Data.data[0]);
        tofLeft(Local_Data.data[1]);
        tofRight(Local_Data.data[2]);
        tofBack(Local_Data.data[3]);
        imuBearing(Local_Data.data[4]);

        botKinematics();
    }


    // Triggers on /TOF_Front post sets linear y speed based on pids
    void tofFront(int tof_front){
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
            double error_y = desired_y - tof_front;
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
        //botKinematics();
    }


    // Triggers on /TOF_Left post sets linear x speed based on pids
    void tofLeft(int tof_left){
        // If desired_x = 0 stop x movement, ingore x sensor, declare arrived
        if(desired_x == 0){
            linear_x = 0;
            arrived_x = SEQUENTIAL_READS;
        } 

        // If desired x is negative use sensor data to calculate pid for x velocity from left sensor
        else if(desired_x < 0){
            double error_x = -desired_x - tof_left;
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
        //botKinematics();
    }


    // Triggers on /TOF_Right post sets linear x speed based on pids
    void tofRight(int tof_right){
        // If desired_x = 0 stop x movement, ingore x sensor, declare arrived
        if(desired_x == 0){
            linear_x = 0;
            arrived_x = SEQUENTIAL_READS;
        }

        // If desired x is positive use sensor data to calculate pid for x velocity from right sensor
        else if(desired_x > 0){
            double error_x = -1 * (desired_x - tof_right);
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
        //botKinematics();
    }

    
    // Backup and liftoff safety stops all wheel movement
    void tofBack(int tof_back){
        // More sensitive back up safety
        if(tof_back > BACKUP_SAFETY && desired_y < 0){
            e_stop = 1;
            // ROS INFO
            #if DEBUG
                ROS_WARN("**********EStop backup");
            #endif
        }

        // Standard lift up to stop safety
        if(tof_back > LIFTUP_SAFETY && e_stop != 1){
            e_stop = 1;
            // ROS INFO
            #if DEBUG
                ROS_WARN("**********EStop pickup");
            #endif
        }
    }


    // Triggers on /IMU_Bearing post sets rotation z spead based on pids and offset
    void imuBearing(int imu_bearing){
        // If bearing_offset is negative it is uninitialized, so set it as offset
        //if(bearing_offset == -1 && desired_z != -1)
        if(imu_wakeup < 0){
            bearing_offset = imu_bearing;
            imu_wakeup++;
            #if DEBUG
                if(imu_wakeup == -1)
                    ROS_WARN("**********bearing offset: %f", bearing_offset);
            #endif
        }
        
        // Find difference or errot between desired and actual
        double error_z = desired_z - (imu_bearing - bearing_offset);

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
        //botKinematics();
    }


    // Takes xy velocity and z rotation and calculates wheel speeds scaled to max speed
    void botKinematics(){
        // Find angle of linear movement for kinematics
        double theta = atan2(linear_y, linear_x);
        double wheel_speeds[3];
        // Make angle positive
        if(theta < 0)
            theta += 6.28;

        // Scale speed based on x, y, and max speeds
        double speed = sqrt(linear_x * linear_x + linear_y * linear_y);
        if(speed > max_speed)
            speed = max_speed;

        // Inverse wheel kinematics
        wheel_speeds[0] = speed * cos(theta) + linear_z;
        wheel_speeds[1] = speed * cos(theta + (2.0 / 3.0 * 3.1415)) + linear_z;
        wheel_speeds[2] = speed * cos(theta - (2.0 / 3.0 * 3.1415)) + linear_z;
        
        // Find max value of wheels
        double max = abs(wheel_speeds[0]);
        if(abs(wheel_speeds[1]) > max)
            max = abs(wheel_speeds[1]); 
        if(abs(wheel_speeds[2]) > max) 
            max = abs(wheel_speeds[2]); 

        // If max value greater than fastest wheel speeds scale all speeds down
        if(max > MAX_SPEED){
            wheel_speeds[0] = wheel_speeds[0] / max * MAX_SPEED;
            wheel_speeds[1] = wheel_speeds[1] / max * MAX_SPEED;
            wheel_speeds[2] = wheel_speeds[2] / max * MAX_SPEED;
        }

        //ROS_INFO("Arrived array: %f, %f, %f", arrived_x, arrived_y, arrived_z);

        // Checks to see if xyz are all stable and happy and we arn't about to die
        if(e_stop == 1 || (max_speed != 0 && arrived_x >= SEQUENTIAL_READS && arrived_y >= SEQUENTIAL_READS && arrived_z >= SEQUENTIAL_READS)){
            // Debug info
            #if DEBUG
                ROS_WARN("**********Bot arrived");
            #endif

            // Ignore sensors and set speed to 0
            max_speed = 0;
        }

        // If max_Speed 0 overide all orders with stop
        if(max_speed == 0){
            // Publish move done
            Move_Done.data = 1;
            Move_Done_pub.publish(Move_Done);
            
            Local_En.data = false;
            Local_En_pub.publish(Local_En);

            wheel_speeds[0] = 0;
            wheel_speeds[1] = 0;
            wheel_speeds[2] = 0;
        }

        // Publish speed to wheels
        //Wheel_Speeds_pub.publish(Wheel_Speeds);
        wheelSpeeds(wheel_speeds);
    }


    // Takes 3 Wheel_Speeds and writes to servos ID 9-11
    void wheelSpeeds(double *wheel_speeds){
        // Creates and assigns array with each byte of message
        uint8_t data_array[4];
        for (int i = 1; i < 4; i++){    // Do for all 3 wheel servos
            int32_t data = (int32_t)(wheel_speeds[i - 1]); // Convert int32 to uint32
            data_array[0] = DXL_LOBYTE(DXL_LOWORD(data));
            data_array[1] = DXL_HIBYTE(DXL_LOWORD(data));
            data_array[2] = DXL_LOBYTE(DXL_HIWORD(data));
            data_array[3] = DXL_HIBYTE(DXL_HIWORD(data));
            groupSyncWrite_wheelGoalVel.addParam((i + 8), data_array);  // Adds message to stack arguments(servo ID, data array of bytes)
        }

        // Sets a timeout for current wheel speeds
        arduino_timeout_time = clock() + ARDUINO_TIMEOUT * 1000;
        // If we are not moving already make timeout infinite
        if(wheel_speeds[0] == 0 && wheel_speeds[1] == 0 && wheel_speeds[2] == 0)
            arduino_timeout_time = -1;
        
        // Sets flag to write to servos outside of callback
        sync_wheel_goal_vel = 1;
    }


    // Writes thenn clears prepared sync write stacks if new availible
    void syncWrite(){
        // Sync write/clear if new arm acc/vel ready
        if(sync_arm_acc_vel){
            int write_result1 = groupSyncWrite_armAcc.txPacket();
            groupSyncWrite_armAcc.clearParam();
            int write_result2 = groupSyncWrite_armVel.txPacket();
            groupSyncWrite_armVel.clearParam();
            sync_arm_acc_vel = 0;

            // Debug info
            #if DEBUG
                ROS_WARN("**********Sync write arm acc/vel result: %d, %d", write_result1, write_result2);
            #endif
        }

        // Sync write/clear if new arm posistions ready
        else if(sync_arm_goal_pos){
            int write_result3 = groupSyncWrite_armGoalPos.txPacket();
            groupSyncWrite_armGoalPos.clearParam();
            sync_arm_goal_pos = 0;
            // Debug info
            #if DEBUG
                ROS_WARN("**********Sync write arm goal result: %d", write_result3);
            #endif
        }

        // Sync write/clear if new misc posistions ready
        else if(sync_misc_goal_pos){
            int write_result4 = groupSyncWrite_miscGoalPos.txPacket();
            //if(!write_result4){
                groupSyncWrite_miscGoalPos.clearParam();
                sync_misc_goal_pos = 0;
                // Sets the misc moving flag to start checking for servos to complete move
                misc_moving = 1;
            //}
            // Debug info
            #if DEBUG
                ROS_WARN("**********Sync write misc goal result: %d", write_result4);
            #endif
        }

        // Sync write/clear if new wheel speeds ready
        else if(sync_wheel_goal_vel){
            int write_result5 = groupSyncWrite_wheelGoalVel.txPacket();
            groupSyncWrite_wheelGoalVel.clearParam();
            sync_wheel_goal_vel = 0;
            // Debug info
            #if DEBUG
                //ROS_WARN("**********Sync write wheel goal result: %d", write_result5);
            #endif
        }
    }
 

// Pub, sub, and ros declarations
private: 
    ros::NodeHandle nh;

    // Publisher ros declarations
    ros::Publisher Feedback_pub;
    ros::Publisher Arm_Angles_pub;
    ros::Publisher Arm_Done_pub;
    ros::Publisher Move_Done_pub;
    ros::Publisher Misc_Done_pub;
    ros::Publisher Task_Space_pub;
    ros::Publisher Local_En_pub;

    // Subscriber ros declarations
    ros::Subscriber Get_Feedback_sub;
    ros::Subscriber Task_Space_sub;
    ros::Subscriber Misc_Angles_sub;
    ros::Subscriber Arm_Angles_sub;
    ros::Subscriber Move_sub;
    ros::Subscriber PIDs_sub;
    ros::Subscriber Local_Data_sub;

    // Publisher variable declarations
    std_msgs::Float32MultiArray Feedback;
    std_msgs::Float32MultiArray Arm_Angles;
    std_msgs::Float32MultiArray Task_Space;

    // Local publisher variables
    std_msgs::Int8 Arm_Done;
    std_msgs::Int8 Move_Done;
    std_msgs::Int8 Misc_Done;
    std_msgs::Bool Local_En;

    // Variable arm values
    int last_arm_angles[8], moving_down = 0, arm_tolerance = 10;

    // Variables for bot movement functions
    double  desired_x = 0, error_x_prev = 0, error_x_cumulative = 0, linear_x = 0, arrived_x = 0,
            desired_y = 0, error_y_prev = 0, error_y_cumulative = 0, linear_y = 0, arrived_y = 0,
            desired_z = -1, error_z_prev = 0, error_z_cumulative = 0, linear_z = 0, arrived_z = 0,
            max_speed = 0, bearing_offset = -1, e_stop = 0, imu_wakeup = -40;

    // Local task space 
    float local_task_space[6];

    // Clock timers
    clock_t next_down_time;
};



// Main loop
int main(int argc, char** argv){

    // Sets up ros node and stuff
    ros::init(argc, argv, "servos");
    ros::NodeHandle nh;
    //ros::MultiThreadedSpinner spinner(0);   // Do i still need this???

    // Create main servo class
    ServoClass ServoObject(&nh);
    // Starts communication and sets start vaules for servos
    ServoObject.initializeServos();

    // Wheel stopped timeout array
    double stop_wheels[] = {0, 0, 0};

    // Main loop
    while (ros::ok())
    {   
        // Sync write if new data is availible
        if(ServoObject.sync_arm_acc_vel || ServoObject.sync_arm_goal_pos || ServoObject.sync_misc_goal_pos || ServoObject.sync_wheel_goal_vel)
            ServoObject.syncWrite();

        // Once triggered by new arm posistion will keep running until close enough to desired posistion
        if(ServoObject.arm_moving)
            ServoObject.armMoving();
        
        // Once triggered by new miscCallback will keep running until close enough to desired posistion
        if(ServoObject.misc_moving)
            ServoObject.miscMoving();
        
        // Every few seconds restart servos if torqued out
        if(ServoObject.restart_check_time < clock())
            ServoObject.readRestart();

        // If no arduino has been received recently, and we are wanting to move turn off wheels
        if(ServoObject.arduino_timeout_time < clock() && ServoObject.arduino_timeout_time != -1)
            ServoObject.wheelSpeeds(stop_wheels);

        ros::spinOnce();
        //spinner.spin();
    }

    // Close ports and stop node
    packetHandler->write4ByteTxOnly(portHandler, SECONDARY_ID, GOAL_VELOCITY_ADDR, STOP);               // Sets velocity to stop
    portHandler->closePort();
    return 0;
}

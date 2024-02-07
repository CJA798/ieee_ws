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

//variables for the movement arrays
int Stop = [0,0,0];
int Go = [0,0,0];
int Backwards = [0,0,0];
int TurnCW = [0,0,0];
int TurnCCW = [0,0,0];
int Go_Right = [0,0,0];
int Go_Left = [0,0,0];

//N


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

          /*
        switch(event){
            case STOP:
            //All motors off
            wheelSpeeds.data = {0,0,0};
            wheel_speed_pub.publish(wheelSpeeds);
            Movement(Stop);

            case BIG_BLOCKS:
            //motors go forward about 10 in


            // Start the timer, Still working on this
            auto start_time = std::chrono::high_resolution_clock::now();
    
            // Perform some task (for demonstration, we'll just sleep for a while)
            Movement(Go_Forward); // Sleep for 3 seconds
    
           // Stop the timer
            auto end_time = std::chrono::high_resolution_clock::now();

            //After a certain amount of time, stop
            Movement(Stop);

            //block collection will take place, announce state to be "ready for collection"

            case 
            */
            

      













        }

        /*
        //THIS WORKS FOR CLOCKWISE TURNS 90 AND 180 NOT 360
void TurnCW(int degrees){  //put in the angle you would like the bot to end up facing on the board (in reference to starting position) 270 -> Left, 180 -> backwards, 360/0 -> straight ahead

   //double turning = desired_orientation + degrees; // degrees can be 90, 180
   //option = 0;
    double difference = current_orientation - desired_orientation;
    if(desired_orientation + degrees > 360){
      degrees = (desired_orientation + degrees) - 360;
      option = 1;
    }
    else {
      degrees = desired_orientation + degrees;
      option = 2;
    }

    switch(option){

      case 1:

    if(current_orientation <= 360 && current_orientation >= 270){
      Movement(0,0,1);
      current_orientation = getOrientationX(&orientationData);
    }
    else if(current_orientation <= degrees){
        Movement(0,0,1);
        current_orientation = getOrientationX(&orientationData);
      }

  else{
     Movement(0,0,0);
     event = PHASE_1;
  }


    break;


    case 2:

     if(current_orientation <= degrees){
        Movement(0,0,1);
        current_orientation = getOrientationX(&orientationData);
      }

  else {
    Movement(0,0,0);
    event = PHASE_1;
  }






    break;

    default:break;

}
}

void TurnCCW(int degrees){  //put in the angle you would like the bot to end up facing on the board (in reference to starting position) 270 -> Left, 180 -> backwards, 360/0 -> straight ahead
    double difference = current_orientation - desired_orientation;

  //  double old_curr = current_orientation; 
    if(desired_orientation - degrees < 0){
      degrees = 360 + (desired_orientation - degrees);
      option = 1;
    }
    else {
      degrees = desired_orientation - degrees;
      option = 2;
    }

    switch(option){
      case 1:
       if(350 >= current_orientation && current_orientation <=90){
      Movement(0,0,-1);
      current_orientation = getOrientationX(&orientationData);
    }
    else if(current_orientation >= degrees){
        Movement(0,0,-1);
        current_orientation = getOrientationX(&orientationData);
      }

  else{
   Movement(0,0,0);
   event = PHASE_1;
  }


      break;


      case 2:
      if(current_orientation >= degrees){
        Movement(0,0,-1);
        current_orientation = getOrientationX(&orientationData);
      }

  else{
   Movement(0,0,0);
   event = PHASE_1;
  }



      break;

      default:break;

    }
        
        
        */

        // Spin and sleep
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
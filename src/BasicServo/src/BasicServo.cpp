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
int Stop[] = {0, 0, 0};
double Go[] = {0, 200, 0};
int Backwards[] = {0, 0, 0};
int TurnCW[] = {0, 0, 0};
int TurnCCW[] = {0, 0, 0};
int Go_Right[] = {0, 0, 0};
int Go_Left[] = {0, 0, 0};
float* V_ang;
float VelocityBot[] = {0,0,0};
float Vlin[] = {0,0,0};




//Nav States
enum class State {
    TEST,
    STOP,
    BIG_BLOCKS,
    SLOPE_DETECT,
    WAIT
};

  float* Movement(double array[]) {


  int radius = 50.8;

  VelocityBot[0] = array[0];  //update global variable for robot local velocity
  VelocityBot[1] = array[1];
  VelocityBot[2] = array[2];


  //update math for wheel velocities
  Vlin[0] = -.5 * VelocityBot[0] - 0.866 * VelocityBot[1] + 105.25 * VelocityBot[2];  //va
  Vlin[1] = -.5 * VelocityBot[0] + 0.866 * VelocityBot[1] + 105.25 * VelocityBot[2];  //vb
  Vlin[2] = VelocityBot[0] + 105.25 * VelocityBot[2];                                 //vc

  Vlin[0] = 60 * (Vlin[0] / radius) / (0.229 * 2 * 3.14);
  Vlin[1] = 60 * (Vlin[1] / radius) / (0.229 * 2 * 3.14);
  Vlin[2] = 60 * (Vlin[2] / radius) / (0.229 * 2 * 3.14);

  return Vlin;



  /*
  dxl.setGoalVelocity(DXL_ID1, Vlin[2]);  //vc
  dxl.setGoalVelocity(DXL_ID2, Vlin[1]);  //vb
  dxl.setGoalVelocity(DXL_ID3, Vlin[0]);  //va
  */
  
}




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
    ros::init(argc, argv, "BasicServo");
    ros::NodeHandle nh;

    // create subscriber objects
    ros::Subscriber front_tof_sub = nh.subscribe("TOF_Front", 10, tofOneCallback);
    ros::Subscriber left_tof_sub = nh.subscribe("TOF_Left", 10, tofTwoCallback);
    ros::Subscriber right_tof_sub = nh.subscribe("TOF_Right", 10, tofThreeCallback);
    ros::Subscriber back_tof_sub = nh.subscribe("TOF_Left", 10, tofFourCallback);
    ros::Subscriber bearing_sub = nh.subscribe("IMU_Bearing", 10, imuBearCallback);
    ros::Subscriber grav_sub = nh.subscribe("IMU_Grav", 10, imuGravCallback);
    ros::Subscriber bot_state_sub = nh.subscribe("State_SM2Nav", 10, stateCallback);

    // create publisher objects
    ros::Publisher nav_state_pub = nh.advertise<std_msgs::String>("State_Nav2SM", 10);
    //ros::Publisher wheel_speed_pub = nh.advertise<std_msgs::Float64MultiArray>("wheel_speeds", 10);
    ros::Publisher wheel_speed_pub = nh.advertise<std_msgs::Float32MultiArray>("Wheel_Speeds", 10);

    // Set the loop rate
    ros::Rate loopRate(10); // 10 Hz

    int event = 0;
    
    //check botState.data to see if we can start
    //publish whether you're moving or not using nav_state_pub
    wheelSpeeds.data.resize(3);



    while (ros::ok()) {
        // update publishing objects and publish them
        


          
        switch(event){
            case 0:
            //test values

              V_ang = Movement(Go);

              wheelSpeeds.data[0]= V_ang[0];
              wheelSpeeds.data[1]= V_ang[1];
              wheelSpeeds.data[2]= V_ang[2];
              wheel_speed_pub.publish(wheelSpeeds);
          
            
            navState.data = "Going";
             nav_state_pub.publish(navState);

            break;
            /*

            case BIG_BLOCKS:
            //motors go forward about 10 in


            // Start the timer, Still working on this
            auto start_time = std::chrono::high_resolution_clock::now();
    
            // Perform some task (for demonstration, we'll just sleep for a while)
            Movement(Go_Forward); // Sleep for 3 seconds
            navState.data = "Moving";
             nav_state_pub.publish(navState);
    
           // Stop the timer
            auto end_time = std::chrono::high_resolution_clock::now();

            //After a certain amount of time, stop
            Movement(Stop);

            //block collection will take place, announce state to be "ready for collection"
             navState.data = "Collection";
             nav_state_pub.publish(navState);

             break;

            case SLOPE_DETECT:
            Go_Forward(tofRight);
            slopeDetect(gravVector);
            //After detecting 2 slopes, stop and start watching the front sensor
            navState.data = "Ramp";
            nav_state_pub.publish(navState);

            Go_Forward(tofFront);
            //Arrive at green area

            TurnCCW(90);

            break;

            case GO_BLUE:
            Go_Forward(tofFront);
            navState.data = "Waiting";
             nav_state_pub.publish(navState);

            break;

            //test jason


            





            
            

      













        }*/
/////////////////////////////////////////////////////////////////////////

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


    //Slope detect function:
   // Values are multiplied by 10
    void slopeDetect(int xVector) {

  if ((xVector > 28) || (xVector < -28)) {
    onSlope = 1;
  }
  if ((onSlope == 1) && (xVector < 3) && (xVector > -3)) {  //if we where just on the slope and are now flat
    slopeCount++;
    onSlope = 0;
  }
}

Go Forward function:
void Go_Forward(int S1, int S2, int S4) {  //go forward specified distance from wall
  if (90 < S2 < 160) {                     //if robot with chosen wall range then go forward
    Movement(0, 200, 0);
  }

  if (S2 >= 160) {        //if robot is too far away from wall move closer
    Movement(100, 0, 0);  //This variable may be too quick, watch it.
  }
  if (S2 <= 90) {  //if robot is too close to wall move away
    Movement(-100, 0, 0);
  }
}
        
        
        */

        // Spin and sleep
        ros::spinOnce();
        loopRate.sleep();
    }

    
  }
  return 0;
}




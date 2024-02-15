#include <string>
#include <cstring>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
//#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#define WAITING_FOR_SM 99


// global variables for wheel speed and robot state
float wheelSpeedOne;
float wheelSpeedTwo;
float wheelSpeedThree;

// global variables for sensor data
int tofFront;
int tofLeft;
int tofRight;
int tofBack;
int bearing;
int gravVector;

//variables for the movement arrays
double Stop[] = {0, 0, 0};
double Go[] = {0, 220, 0};
double Backwards[] = {0, -50, 0};
double TurnCW[] = {0, 0, 1};
double TurnCCW[] = {0, 0, -1};
double Go_Right[] = {150, 150, 0};
double Go_Left[] = {-150, 150, 0};

//Arrays for Wheel Speeds
float* V_ang;
float VelocityBot[] = {0,0,0};
float Vlin[] = {0,0,0};

//Variables for movement functions
int desired_orientation = 0;
int current_orientation = 0;

// Initialize the event to wait until main SM is ready
//int event = WAITING_FOR_SM; // event = 99
int event = 0;
int onSlope = 0;
int slopeCount = 0;
int center = 0;
std::string navString_input = "Waiting";


class NavClass{
  public:
  // Create state variable
  std_msgs::String botState;
  NavClass(ros::NodeHandle* nodehandle){ 
        nh = *nodehandle;        

        // Resize Publisher Arrays
        wheelSpeeds.data.resize(3);

        


        // create subscriber objects
    front_tof_sub = nh.subscribe("TOF_Front", 10, &NavClass::tofOneCallback, this);
    left_tof_sub = nh.subscribe("TOF_Left", 10, &NavClass::tofTwoCallback, this);
    right_tof_sub = nh.subscribe("TOF_Right", 10, &NavClass::tofThreeCallback, this);
    back_tof_sub = nh.subscribe("TOF_Back", 10, &NavClass::tofFourCallback, this);
    bearing_sub = nh.subscribe("IMU_Bearing", 10, &NavClass::imuBearCallback, this);
    grav_sub = nh.subscribe("IMU_Grav", 10, &NavClass::imuGravCallback, this);
    bot_state_sub = nh.subscribe("State_SM2Nav", 10, &NavClass::stateCallback, this);

    // create publisher objects
    nav_state_pub = nh.advertise<std_msgs::String>("State_Nav2SM", 10);
    wheel_speed_pub = nh.advertise<std_msgs::Float32MultiArray>("Wheel_Speeds", 10);
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


  void publishSpeedsAndState(float* V_ang, const std::string& str){
    wheelSpeeds.data[0]= V_ang[2];
    wheelSpeeds.data[1]= V_ang[1];
    wheelSpeeds.data[2]= V_ang[0];
    wheel_speed_pub.publish(wheelSpeeds);

    navState.data = str;
    nav_state_pub.publish(navState);
  }

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


}



void Turn_CCW(int degrees){  //put in the angle you would like the bot to end up facing on the board (in reference to starting position) 270 -> Left, 180 -> backwards, 360/0 -> straight ahead
   // double difference = current_orientation - desired_orientation;
   int option =  0;

   
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
       if(350 >= bearing && bearing <=90){
      navString_input = "Turning CCW";
      publishSpeedsAndState(Movement(TurnCCW), navString_input);
    }
    else if(bearing >= degrees + 10){
        navString_input = "Turning CCW";
        publishSpeedsAndState(Movement(TurnCCW), navString_input);

      }

  else{
    navString_input = "Waiting";
    publishSpeedsAndState(Movement(Stop), navString_input);
  }


      break;


      case 2:
      if(bearing >= degrees + 10){
        navString_input = "Turning CCW";
        publishSpeedsAndState(Movement(TurnCCW), navString_input);
      }

  else{
    navString_input = "Waiting";
    publishSpeedsAndState(Movement(Stop), navString_input);
    center = tofRight;
    desired_orientation = bearing;
  }



      break;

      default:break;

    }
}

void Turn_CW(int degrees){  //put in the angle you would like the bot to end up facing on the board (in reference to starting position) 270 -> Left, 180 -> backwards, 360/0 -> straight ahead

    int option = 0;

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

    if(bearing <= 360 && bearing >= 270){
      navString_input = "Turning CW";
      publishSpeedsAndState(Movement(TurnCW), navString_input);
      
    }
    else if(current_orientation <= degrees - 10){
      navString_input = "Turning CW";
      publishSpeedsAndState(Movement(TurnCW), navString_input);
        
      }

  else{
    navString_input = "Waiting";
    publishSpeedsAndState(Movement(Stop), navString_input);
  }


    break;


    case 2:

     if(bearing <= degrees -10){
      navString_input = "Turning CW";
      publishSpeedsAndState(Movement(TurnCW), navString_input);
        
      }

  else {
    navString_input = "Waiting";
    publishSpeedsAndState(Movement(Stop), navString_input);
  }






    break;

    default:break;

}
}



// Values are multiplied by 10
    void slopeDetect(int xVector) {

  if ((xVector > 33) || (xVector < -32)) {
    onSlope = 1;
  }
  if ((onSlope == 1) && (xVector < 3) && (xVector > -3)) {  //if we where just on the slope and are now flat
    slopeCount++;
    onSlope = 0;
  }
}

void Go_Forward(int sensor) {  //go forward specified distance from wall
  if (center - 20 < tofRight < center + 20) {                     //if robot with chosen wall range then go forward
    navString_input = "Going Forward";
    publishSpeedsAndState(Movement(Go), navString_input);
  }

  if (center +20  < tofRight) {        //if robot is too far away from wall move closer
    navString_input = "Moving Right";
    publishSpeedsAndState(Movement(Go_Right), navString_input);  //This variable may be too quick, watch it.
  }
  if (center -20 > tofRight) {  //if robot is too close to wall move away
    navString_input = "Moving Left";
    publishSpeedsAndState(Movement(Go_Left), navString_input);
  }
}







private:
    ros::NodeHandle nh;
    // create subscriber objects
    ros::Subscriber front_tof_sub;
    ros::Subscriber left_tof_sub;
    ros::Subscriber right_tof_sub;
    ros::Subscriber back_tof_sub;
    ros::Subscriber bearing_sub;
    ros::Subscriber grav_sub;
    ros::Subscriber bot_state_sub;
    // create publisher objects
    ros::Publisher nav_state_pub;
    ros::Publisher wheel_speed_pub;
    // create variables for publishing
    std_msgs::Float32MultiArray wheelSpeeds;
    std_msgs::String navState;
};

class Timer {
public:
    Timer() : start_time_(std::chrono::steady_clock::now()) {}

    void reset() {
        start_time_ = std::chrono::steady_clock::now();
    }

    double elapsed() const {
        auto end_time = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count() / 1000.0;
    }

private:
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
};


int main(int argc, char **argv) {
    // Initialize ROS node and name node
    ros::init(argc, argv, "navigation_SM");
    ros::NodeHandle nh;
    // Set the loop rate
    ros::Rate loopRate(10); // 10 Hz
    // Create a NavClass object
    NavClass nav_obj(&nh);
    Timer timer;

    int bottom_sensor = 0;



    
    //check botState.data to see if we can start
    //publish whether you're moving or not using nav_state_pub
    

    while (ros::ok()) {
        // update publishing objects and publish them
        switch(event){
          //Initial states for the bot
            case 0:

     //timer for 5 seconds to let the sensors boot up 
    if (timer.elapsed() > 6.0) {
        std::cout << "5 seconds elapsed" << std::endl;
          //initial state
            navString_input = "Waiting";
            nav_obj.publishSpeedsAndState(nav_obj.Movement(Stop), navString_input);
            //obtain desired direction which will be the initial heading of the bot
            desired_orientation = bearing;
            center = tofRight;
            bottom_sensor = tofBack;
            std::cout<< "center: " << center << std::endl;
            

            event++;
    }

            break;  
            
            
            //initial movement going straight, detects the right TOF sensor
            case 1:
            nav_obj.Go_Forward(tofRight);
            event++;

            break;


            //going up the 2 slopes and arriving at blue area
            case 2:
            nav_obj.slopeDetect(gravVector);

            
             if ((slopeCount == 2) && (tofFront < 180)) {
              navString_input = "Waiting";
                nav_obj.publishSpeedsAndState(nav_obj.Movement(Stop), navString_input);
                current_orientation = bearing;
                event++;
                break;
             }
             
             event--;
            


            break;



            //leaving blue area, block drop location
            case 3:

            nav_obj.Turn_CCW(90); 
            //initialize values to keep straight, already inside of the Turn_CCW function
            event++;

            break;
            
            //green area arrival, gas tank collection
            
            case 4:
            
            nav_obj.Go_Forward(tofRight);
            if(tofFront < 260){ //check if its at the specific location for collection
              navString_input = "Collection";
              nav_obj.publishSpeedsAndState(nav_obj.Movement(Stop), navString_input);
              event++;

            }

            break;


            //prepare to go up the third ramp/leaving the green area
            case 5:
            nav_obj.Turn_CCW(90);
            event++;

            break;

            //go up the third slope
            case 6:
            if(slopeCount == 3){
              navString_input = "Waiting";
              nav_obj.publishSpeedsAndState(nav_obj.Movement(Stop), navString_input);
              event++;
            }

            break;

            //turn around for bridge drop
            case 7:
            nav_obj.Turn_CCW(180);
            event++;

            break;

            //detect crater and drop bridge
            case 8:
            nav_obj.publishSpeedsAndState(nav_obj.Movement(Backwards), navString_input);
            if(tofBack > bottom_sensor){
              navString_input = "Crater";
              nav_obj.publishSpeedsAndState(nav_obj.Movement(Stop), navString_input);
              //drop bridge here
              event++;
            }


            break;

            //Go Forward for a ceratin amount of time in order to drop the bridge
            case 9:
            Timer timer2;
            //after time elapsed tell bot to stop
            if(timer2.elapsed() > 1.5){
              navString_input = "Waiting";
              nav_obj.publishSpeedsAndState(nav_obj.Movement(Stop), navString_input);
              event++;
            } 
            //else keep going forward
            else{
             nav_obj.publishSpeedsAndState(nav_obj.Movement(Go), navString_input)
            }
            break;


            //Go backwards on the bridge
            case 10:
            nav_obj.publishSpeedsAndState(nav_obj.Movement(Backwards), navString_input);
            event++;

            break;


            //After a certain amount of time has passed, stop, turn around and go forward
            case 11:
            Timer timer3;
            if(timer3.elapsed() > 5.0){
              navString_input = "Waiting";
              nav_obj.publishSpeedsAndState(nav_obj.Movement(Stop), navString_input);
              event++;
            } 
            else{
             nav_obj.publishSpeedsAndState(nav_obj.Movement(Backwards), navString_input);
            }
            break;

            //turning around
            case 12:
            nav_obj.Turn_CW(180);
            nav_obj.Go_Forward(tofRight);
            event++;
            break;

            case 13:
            if(tofFront < 270){
              
              if(tofFront > 10){
              navString_input = "Going Right";
              nav_obj.publishSpeedsAndState(nav_obj.Movement(Go_Right), navString_input);
              }
              else{
                navString_input = "Waiting";
                nav_obj.publishSpeedsAndState(nav_obj.Movement(Stop), navString_input);
                event++;
              }
              
            }




            case WAITING_FOR_SM:
            if (nav_obj.botState.data == "Start") {
                event = 0;
            }
            break;

            default:break;
        
        std::cout << "End of loop" << std::endl;
        
    }
    // Spin and sleep
        ros::spinOnce();
        loopRate.sleep();
    
  }
  return 0;
}


//Functions for state machine

        
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
*/



//jason commit test2

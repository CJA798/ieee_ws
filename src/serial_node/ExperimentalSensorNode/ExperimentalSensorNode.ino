#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "functions.h"
#include "macros.h"

ros::NodeHandle nh;

// Globals
unsigned int event = WAITING_FOR_NH; 

// Init Variables
bool initState = 0;
bool initStarted = 0;

// Init Subscriber
void initCb(const std_msgs::Bool& init_msg){
  initState = init_msg.data;
}
ros::Subscriber<std_msgs::Bool> initsub("Init_State", &initCb );


// LED Variables
int detVal = 700;
bool botCal = 0;
int ledVal = 0;

// LED Publisher
std_msgs::Bool ledDet;
ros::Publisher ledpub("LED_State", &ledDet);


// IMU Variables
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28, &Wire);
adafruit_bno055_offsets_t calibrationData;
sensors_event_t gravityData, orientationData;
unsigned long last_imu_pub_time = 0;

// IMU Publishers
std_msgs::Int16 imuBearing;
std_msgs::Int16 imuGrav;
ros::Publisher imubpub("IMU_Bearing", &imuBearing);
ros::Publisher imugpub("IMU_Grav", &imuGrav);


// TOF Variables
VL53L1X TOF1;
VL53L1X TOF2;
VL53L1X TOF3;
VL53L1X TOF4;
uint8_t xshut1 = 2;
uint8_t xshut2 = 4;
uint8_t xshut3 = 3;
uint8_t xshut4 = 5;
int T1 = 0;
int T2 = 0;
int T3 = 0;
int T4 = 0;
int T1prev = 0;
int T2prev = 0;
int T3prev = 0;
unsigned long last_tof_pub_time = 0;


// TOF Publishers
std_msgs::Int16 tof1Data;
std_msgs::Int16 tof2Data;
std_msgs::Int16 tof3Data;
std_msgs::Int16 tof4Data;
ros::Publisher tof1pub("TOF_Front", &tof1Data);
ros::Publisher tof2pub("TOF_Left", &tof2Data);
ros::Publisher tof3pub("TOF_Right", &tof3Data);
ros::Publisher tof4pub("TOF_Back", &tof4Data);


// Setup heartbeat publisher
std_msgs::Empty heartbeat_msg;
ros::Publisher heartbeat_pub("Heartbeat", &heartbeat_msg);
unsigned long last_heartbeat_time = 0;

void setup() {
  // Set up communication
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(500000);


  // Initialize ROS node
  nh.initNode();

  // Advertise publishers and subscribers
  nh.advertise(heartbeat_pub);
  nh.advertise(imubpub);
  nh.advertise(imugpub);
  nh.advertise(tof1pub);
  nh.advertise(tof2pub);
  nh.advertise(tof3pub);
  nh.advertise(tof4pub);
  nh.advertise(ledpub);
  nh.subscribe(initsub);


  // Initialize pin for LED detection
  initLEDDetector();

  // Initialize IMU
  if (!IMU.begin())
  {
    nh.logerror("Could not find a valid BNO055 sensor, check wiring, address, sensor ID, and system status");
    while (1);
  }

  // Set up IMU  
  // Set mode
  /*
  We may want to start with OPERATION_MODE_COMPASS and see if it performs well.
  If we need more stability or faster response, we can switch to OPERATION_MODE_NDOF,
  keeping in mind that it might require more processing power and calibration effort.
  */
  //IMU.setMode(OPERATION_MODE_COMPASS);
  IMU.setMode(OPERATION_MODE_NDOF);

  // Upload calibration data
  calibrationData.accel_offset_x = 16908;
  calibrationData.accel_offset_y = 352;
  calibrationData.accel_offset_z = 256;
  calibrationData.gyro_offset_x = 20001;
  calibrationData.gyro_offset_y = 1;
  calibrationData.gyro_offset_z = 8564;
  calibrationData.mag_offset_z = 256;
  calibrationData.mag_offset_x = 20001;
  calibrationData.mag_offset_y = 1;
  calibrationData.accel_radius = 17409;
  calibrationData.mag_radius = 0;

  IMU.setSensorOffsets(calibrationData);

  // Use external crystal for better accuracy
  // Crystal must be configured AFTER loading calibration data into BNO055
  IMU.setExtCrystalUse(true);


  // Setup TOF sensors
  // Toggle xshut pins for reset
  setXShutPins();
  setAllTOFAddresses();

  // set previous TOF readings for LPF
  T1prev = TOF1.read();
  T2prev = TOF2.read();
  T3prev = TOF3.read();

  // publish LED state 
  ledDet.data = 0;
  ledpub.publish(&ledDet);

  // initial bearing publish
  sensors_event_t gravityData, orientationData;
  IMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  imuBearing.data = orientationData.orientation.x;
  imubpub.publish(&imuBearing);
  
  event = WAITING_FOR_NH;
}

void loop() {
   switch(event){
      case WAITING_FOR_NH:
        // If the node handle is connected, move to the next state
        if (nh.connected())
        {
          event = CLEAR;
          nh.loginfo("Arduino node handle connected");
        }
        break;

      case CLEAR:
        // Clear all flags and variables
        initState = 0;
        initStarted = 0;

        detVal = 700;
        botCal = 0;
        ledVal = 0;

        T1 = 0;
        T2 = 0;
        T3 = 0;
        T4 = 0;

        T1prev = 0;
        T2prev = 0;
        T3prev = 0;

        nh.logwarn("Cleared all flags and variables");
        event = IMU_SETUP;
        break;

      case IMU_SETUP:
        // If the IMU is fully calibrated, move to the next state
        if (IMU.isFullyCalibrated())
        {
          /* Display calibration status for each sensor. */
          uint8_t system, gyro, accel, mag = 0;
          IMU.getCalibration(&system, &gyro, &accel, &mag);
          
          char message[128]; // Assuming a maximum message length of 128 characters
          sprintf(message, "CALIBRATION: Sys=%d Gyro=%d Accel=%d Mag=%d", system, gyro, accel, mag);
          nh.logwarn(message);

          nh.loginfo("IMU is fully calibrated");
          event = DETECT_LED;
        }
        // If the IMU is not fully calibrated, run the calibration function
        else
        {
          nh.logwarn("IMU needs to be recalibrated");
          calibrateIMU();
        }
        break;

      case DETECT_LED:
        // Check if ROS is initialized
        if(initState && !initStarted){
          digitalWrite(red, HIGH);
          digitalWrite(green, LOW);
          digitalWrite(blue, LOW);
          initStarted = 1;
          nh.loginfo("Init Started");
        }
        // Check if bot is ready
        //Serial.println(analogRead(A0));
        if(!botCal && digitalRead(startButton) && initState){
          botCal = 1;
          detVal = analogRead(A0) + detValOffset;
          digitalWrite(red, LOW);
          digitalWrite(green, HIGH);
          digitalWrite(blue, LOW);
          nh.loginfo("Bot calibrated");
        }

        // Check if LED is detected
        if (!ledDet.data && (analogRead(0) > detVal) && botCal) {
          ledDet.data = 1;
          ledpub.publish(&ledDet);
          event = TEST;
          nh.loginfo("LED Detected");
        }

      case TEST:
        // Check if it's time to take a new IMU reading
        if (millis() - last_imu_pub_time >= IMU_READ_INTERVAL) {

          // Read IMU data
          IMU.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
          IMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

          // Prepare IMU data
          imuGrav.data = gravityData.acceleration.x * 10;
          imuBearing.data = orientationData.orientation.x;

          // Publish IMU data
          imubpub.publish(&imuBearing);
          imugpub.publish(&imuGrav);

          // Update last IMU publish time
          last_imu_pub_time = millis();
        }

        // Check if it's time to take a new TOF reading
        if (millis() - last_tof_pub_time >= TOF_READ_INTERVAL) {
          // Read TOF data
          T1 = (alpha * TOF1.read()) + ((1 - alpha) * T1prev);
          T2 = (alpha * TOF2.read()) + ((1 - alpha) * T2prev);
          T3 = (alpha * TOF3.read()) + ((1 - alpha) * T3prev);
          T4 = TOF4.read();
          tof1Data.data = T1;
          tof2Data.data = T2;
          tof3Data.data = T3;
          tof4Data.data = T4;
          
          // Publish data
          tof1pub.publish(&tof1Data);
          tof2pub.publish(&tof2Data);
          tof3pub.publish(&tof3Data);
          tof4pub.publish(&tof4Data);

          // Update last TOF publish time
          last_tof_pub_time = millis();

          // Set previous TOF data
          T1prev = T1;
          T2prev = T2;
          T3prev = T3;
        }

        break;


      default: break;

    }
  

  // Publish heartbeat
  if (millis() - last_heartbeat_time > HEARTBEAT_PUB_INTERVAL) {
    heartbeat_pub.publish(&heartbeat_msg);
    last_heartbeat_time = millis();
  }


  // Publish TOF_Back
  T4 = TOF4.read();
  tof4Data.data = T4;
  tof4pub.publish(&tof4Data);


  // Handle ROS communication
  nh.spinOnce();
}

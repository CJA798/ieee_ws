// macros
#define detValOffset 150  // value above ambient light to trigger led
#define startButton A2
#define red 6
#define green 7
#define blue 8
#define alpha 0.7 //0.15

// IMU
#define BNO055_SAMPLERATE_DELAY_MS 100

// TOF
#define CONTINUOUS_TIME 10
#define TOF1_TIMING_BUDGET 200
#define TOF2_TIMING_BUDGET 200
#define TOF3_TIMING_BUDGET 100
#define TOF4_TIMING_BUDGET 100
#define TOF1_ADDRESS 42
#define TOF2_ADDRESS 43
#define TOF3_ADDRESS 44
#define TOF4_ADDRESS 45

// Publish intervals
#define TOF_READ_INTERVAL 50
#define IMU_READ_INTERVAL 100
#define HEARTBEAT_PUB_INTERVAL 500

// State machine states
#define WAITING_FOR_NH 0
#define IMU_SETUP 1
#define TOF_SETUP 2

#define TEST 99

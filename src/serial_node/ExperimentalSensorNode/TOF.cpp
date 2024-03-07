#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include <VL53L1X.h>
#include "functions.h"
#include "macros.h"

// Globals
extern ros::NodeHandle nh;
extern VL53L1X TOF1;
extern VL53L1X TOF2;
extern VL53L1X TOF3;
extern VL53L1X TOF4;
extern uint8_t xshut1;
extern uint8_t xshut2;
extern uint8_t xshut3;
extern uint8_t xshut4;

void setXShutPins()
{
    pinMode(xshut1, OUTPUT);
    pinMode(xshut2, OUTPUT);
    pinMode(xshut3, OUTPUT);
    pinMode(xshut4, OUTPUT);
    digitalWrite(xshut1, LOW);
    digitalWrite(xshut2, LOW);
    digitalWrite(xshut3, LOW);
    digitalWrite(xshut4, LOW);
}


void setAllTOFAddresses()
{
    pinMode(xshut1, INPUT);
    delay(10);
    TOF1.init();
    TOF1.setDistanceMode(VL53L1X::Long);
    TOF1.setMeasurementTimingBudget(TOF1_TIMING_BUDGET);
    TOF1.startContinuous(CONTINUOUS_TIME);
    TOF1.setAddress(TOF1_ADDRESS);   

    pinMode(xshut2, INPUT);
    delay(10);
    TOF2.init();
    TOF2.setDistanceMode(VL53L1X::Long);
    TOF2.setMeasurementTimingBudget(TOF2_TIMING_BUDGET);
    TOF2.startContinuous(CONTINUOUS_TIME);
    TOF2.setAddress(TOF2_ADDRESS);    

    pinMode(xshut3, INPUT);
    delay(10);
    TOF3.init();
    TOF3.setDistanceMode(VL53L1X::Short);
    TOF3.setMeasurementTimingBudget(TOF3_TIMING_BUDGET);
    TOF3.startContinuous(CONTINUOUS_TIME);
    TOF3.setAddress(TOF3_ADDRESS);   

    pinMode(xshut4, INPUT);
    delay(10);
    TOF4.init();
    TOF4.setDistanceMode(VL53L1X::Short);
    TOF4.setMeasurementTimingBudget(TOF4_TIMING_BUDGET);
    TOF4.startContinuous(CONTINUOUS_TIME);
    TOF4.setAddress(TOF4_ADDRESS);
}

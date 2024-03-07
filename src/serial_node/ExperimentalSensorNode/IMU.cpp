#include <ros.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "functions.h"
#include "macros.h"

// Globals
extern ros::NodeHandle nh;
extern Adafruit_BNO055 IMU;
extern adafruit_bno055_offsets_t calibrationData;

void showCalibrationValues()
{
    IMU.getSensorOffsets(calibrationData);
    
    char message[256]; // Assuming a maximum message length of 256 characters

    sprintf(message, "calibrationData.accel_offset_x = %d;", calibrationData.accel_offset_x);
    nh.loginfo(message);
    sprintf(message, "calibrationData.accel_offset_y = %d;", calibrationData.accel_offset_y);
    nh.loginfo(message);
    sprintf(message, "calibrationData.accel_offset_z = %d;", calibrationData.accel_offset_z);
    nh.loginfo(message);
    sprintf(message, "calibrationData.gyro_offset_x = %d;", calibrationData.gyro_offset_x);
    nh.loginfo(message);
    sprintf(message, "calibrationData.gyro_offset_y = %d;", calibrationData.gyro_offset_y);
    nh.loginfo(message);
    sprintf(message, "calibrationData.gyro_offset_z = %d;", calibrationData.gyro_offset_z);
    nh.loginfo(message);
    sprintf(message, "calibrationData.mag_offset_z = %d;", calibrationData.mag_offset_z);
    nh.loginfo(message);
    sprintf(message, "calibrationData.mag_offset_x = %d;", calibrationData.mag_offset_x);
    nh.loginfo(message);
    sprintf(message, "calibrationData.mag_offset_y = %d;", calibrationData.mag_offset_y);
    nh.loginfo(message);
    sprintf(message, "calibrationData.accel_radius = %d;", calibrationData.accel_radius);
    nh.loginfo(message);
    sprintf(message, "calibrationData.mag_radius = %d;", calibrationData.mag_radius);
    nh.loginfo(message);
}


void calibrateIMU()
{
    /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    IMU.getCalibration(&system, &gyro, &accel, &mag);
    
    char message[128]; // Assuming a maximum message length of 128 characters
    sprintf(message, "CALIBRATION: Sys=%d Gyro=%d Accel=%d Mag=%d", system, gyro, accel, mag);
    nh.loginfo(message);
    
    if (system == 3 && gyro == 3 && mag == 3)
    {
        showCalibrationValues();

        // Save calibration data
        IMU.setSensorOffsets(calibrationData);
        nh.loginfo("IMU calibration data uploaded");

        // Use external crystal for better accuracy
        // Crystal must be configured AFTER loading calibration data into BNO055
        IMU.setExtCrystalUse(true);
        nh.loginfo("IMU crystal set to external");
    }

    delay(1000);
}

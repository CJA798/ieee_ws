#include <Arduino.h>
#include <Wire.h>
#include <ros.h>
#include "functions.h"
#include "macros.h"

void initLEDDetector()
{
    pinMode(A0, INPUT);
    pinMode(startButton, INPUT_PULLUP);
    pinMode(red, OUTPUT);
    pinMode(green, OUTPUT);
    pinMode(blue, OUTPUT);
}
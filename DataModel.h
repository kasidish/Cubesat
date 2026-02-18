#ifndef DATA_MODEL_H
#define DATA_MODEL_H

#include <Arduino.h>
#include <FreeRTOS.h>

// FEATURE SWITCHES
#define ENABLE_WIFI    1
#define ENABLE_CAMERA  1
#define ENABLE_SD      1
#define ENABLE_INA226  0 // Set to 1 if hardware acts up
#define ENABLE_RTC     0 // Set to 1 if hardware acts up
#define ENABLE_GPS     0 // Set to 1 if hardware acts up

struct MeasurementData {
    char timestamp[32];
    float vin, iin, pin;
    float vout, iout, pout;
    float efficiency;
    double lat, lng;
    int adcValue;
    float adcVoltage;
};

#endif

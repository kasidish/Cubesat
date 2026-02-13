#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include "INA226.h"
#include <TinyGPS++.h>
#include <RTClib.h>

// Data structure for measurements
struct MeasurementData {
  char timestamp[32];
  float vin, iin, pin;
  float vout, iout, pout;
  float efficiency;
  double lat, lng;
};

class SensorManager {
public:
    SensorManager();
    void begin();
    MeasurementData readData();
    void makeTimestamp(char* out, size_t outSize);
    
private:
    // I2C Pins
    static const int PIN_I2C_SDA = 1;
    static const int PIN_I2C_SCL = 2;
    static const uint8_t ADDR_INA_IN = 0x40;
    static const uint8_t ADDR_INA_OUT = 0x41;

    // GPS Pins
    static const int PIN_GPS_RX = 21;
    static const int PIN_GPS_TX = 47;

    INA226 ina_in;
    INA226 ina_out;
    RTC_DS3231 rtc;
    TinyGPSPlus gps;
    HardwareSerial gpsSerial;
    
    bool rtcOK;
    bool inaOK;
    
    void makeTimestamp(char* out, size_t outSize);
};

#endif

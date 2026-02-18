#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H

#include "DataModel.h"
#include <Wire.h>
#include <TinyGPS++.h>
#include <RTClib.h>
#include "INA226.h"

// I2C Pins
#define I2C_SDA 41
#define I2C_SCL 42

// GPS Pins
#define GPS_RX_PIN 21
#define GPS_TX_PIN 47

// ADC Pin
#define SENSOR_ADC_PIN 3

class SensorService {
public:
    SensorService();
    void begin();
    MeasurementData getLatestData(); 

private:
    static void task(void* param);
    void loop();
    void makeTimestamp(char* out, size_t outSize);

    INA226 ina_in;
    INA226 ina_out; // 0x41
    RTC_DS3231 rtc;
    TinyGPSPlus gps;
    HardwareSerial gpsSerial;

    bool inaOK;
    bool rtcOK;

    MeasurementData latest;
    SemaphoreHandle_t mutex;
    QueueHandle_t* dataQueuePtr; // Pointer to the main queue
    
public:
    void setDataQueue(QueueHandle_t* q) { dataQueuePtr = q; }
};

#endif

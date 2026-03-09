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

// SoC Config (Li-ion 2S 3200mAh)
#define BATT_CAPACITY_MAH   3200.0f
#define BATT_FULL_V         8.4f
#define BATT_EMPTY_V        6.0f
#define OCV_IDLE_THRESHOLD  0.050f  // A - below this is considered idle for OCV correction


// GPS Pins
#define GPS_RX_PIN 21 // GPS TX -> ESP RX
#define GPS_TX_PIN 47 // GPS RX -> ESP TX

// ADC Pins — ordered by physical socket: Pin37→38→39→40
// ADC0=GPIO3(Pin37), ADC1=GPIO2(Pin38), ADC2=GPIO14(Pin39), ADC3=GPIO1(Pin40)
const int ADC_PINS[4] = {3, 2, 9, 1};

class SensorService {
public:
    SensorService();
    void begin();
    MeasurementData getLatestData(); 
    void syncTime(struct tm* timeinfo);

private:
    static void task(void* param);
    void loop();
    void makeTimestamp(char* out, size_t outSize);
    void updateSoC(MeasurementData& d);
    float getSoCFromVoltage(float voltage);


    INA226 ina_in; // 0x41
    INA226 ina_out; // 0x44
    RTC_DS3231 rtc;
    TinyGPSPlus gps;
    HardwareSerial gpsSerial;

    bool inaInOK;
    bool inaOutOK;
    bool rtcOK;

    MeasurementData latest;
    SemaphoreHandle_t mutex;
    QueueHandle_t* dataQueuePtr; // Pointer to the main queue
    
    // GPS NMEA Parsing State
    String nmeaSentence;
    int totalSatsInView;

    // SoC State
    float socAccum;        // Current SoC %
    unsigned long lastSocMs;
    bool socInitialized;

    float adcFiltered[4];
    unsigned long bootTimeMs;
    
public:
    void setDataQueue(QueueHandle_t* q) { dataQueuePtr = q; }
};

#endif

#ifndef DATA_MODEL_H
#define DATA_MODEL_H

#include <Arduino.h>
#include <FreeRTOS.h>

// FEATURE SWITCHES
#define ENABLE_WIFI    1
#define ENABLE_CAMERA  1
#define ENABLE_SD      1
#define ENABLE_MQTT    1
#define ENABLE_INA226  0 // Set to 1 if hardware acts up
#define ENABLE_RTC     0 // Set to 1 if hardware acts up
#define ENABLE_GPS     0 // Set to 1 if hardware acts up

// WI-FI CONFIGURATION
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// MQTT CONFIGURATION
#define MQTT_BROKER "broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC "cubesat/telemetry"

struct MeasurementData {
    char timestamp[32];
    float vin, iin, pin;
    float vout, iout, pout;
    float efficiency;
    double lat, lng;
    int adcValues[4];
    float adcVoltages[4];
    float adcNormalized[4];
    float estimatedBatteryPct;
};

// System Modes
enum OperationMode {
    MODE_SENSOR,    // Only read sensors, send data
    MODE_CAMERA,    // Only use camera, no sensors
    MODE_SLEEP      // Pause sensors and camera, wait for wakeup command
};

extern OperationMode currentSystemMode;

#endif

#ifndef DATA_MODEL_H
#define DATA_MODEL_H

#include <Arduino.h>
#include <FreeRTOS.h>

// FEATURE SWITCHES
#define ENABLE_WIFI    1
#define ENABLE_CAMERA  1
#define ENABLE_SD      1
#define ENABLE_MQTT    1
#define ENABLE_WIFI_ENTERPRISE 1 // Set to 1 to use WPA2 Enterprise (eduroam)
#define ENABLE_MQTT_TLS 1 // Set to 1 to bypass port 1883 blocking using port 8883 (TLS)

#define ENABLE_INA226  1 // Set to 1 if hardware acts up
#define ENABLE_RTC     1 // Set to 1 if hardware acts up
#define ENABLE_GPS     1 // Set to 1 if hardware acts up

// WI-FI CONFIGURATION (Standard WPA2 Personal)
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// WI-FI ENTERPRISE CONFIGURATION (WPA2 Ent)
#define EAP_IDENTITY "username@university.ac.th"
#define EAP_USERNAME "username@university.ac.th"
#define EAP_PASSWORD "password"

// MQTT CONFIGURATION
#define MQTT_BROKER "broker.hivemq.com"

#if ENABLE_MQTT_TLS
    #define MQTT_PORT 8883
#else
    #define MQTT_PORT 1883
#endif

#define MQTT_TOPIC "cubesat/telemetry"

struct MeasurementData {
    char timestamp[32];
    char snrData[128];
    float vin, iin, pin;
    float vout, iout, pout;
    float efficiency;
    double lat, lng;
    int satellites;
    int adcValues[4];
    float battSoC; // Battery State of Charge (%)
};

// System Modes
enum OperationMode {
    MODE_SENSOR,    // Only read sensors, send data
    MODE_CAMERA,    // Only use camera, no sensors
    MODE_SLEEP      // Pause sensors and camera, wait for wakeup command
};

extern OperationMode currentSystemMode;

#endif

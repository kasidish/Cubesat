#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <WebServer.h>
#include "SensorManager.h"
#include "CameraManager.h"

class WifiManager {
public:
    WifiManager();
    void begin(const char* ssid, const char* password,
               SensorManager* sensors, CameraManager* cam,
               SemaphoreHandle_t camMutex,
               SemaphoreHandle_t latestMutex,
               MeasurementData* latestData,
               volatile bool* captureFlag,
               volatile uint32_t* photoCounter);
               
    void handleClient();

private:
    WebServer server;
    
    // External references
    SensorManager* _sensors;
    CameraManager* _cam;
    SemaphoreHandle_t _camMutex;
    SemaphoreHandle_t _latestMutex;
    MeasurementData* _latestData;
    volatile bool* _captureFlag;
    volatile uint32_t* _photoCounter;

    // Handlers
    void handleRoot();
    void handleJPG();
    void handleCapture();
    void handleJSON();
};

#endif

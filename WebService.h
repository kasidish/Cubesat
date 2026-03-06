#ifndef WEB_SERVICE_H
#define WEB_SERVICE_H

#include <WiFi.h>
#include <WebServer.h>
#include "DataModel.h"
#include "SensorService.h"
#include "CameraService.h"

class WebService {
public:
    WebService();
    void begin(SensorService* sensors, CameraService* cam);
    void update(); // Call in loop() or task

private:
    WebServer server;
    SensorService* sensors;
    CameraService* camera;

    void handleRoot();
    void handleJSON();
    void handleJPG();
    void handleCapture();
    void handleStatus();
    void handleSetMode();
};

#endif

#include <Arduino.h>
#include <WiFi.h>

#include "DataModel.h"
#include "SensorService.h"
#include "TelemetryService.h"
#include "WebService.h"

SharedData shared;
SensorService sensors;
TelemetryService telemetry;
WebService web;

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32-OOP", "12345678");

    shared.begin();

    sensors.begin(&shared);
    telemetry.begin(&shared);
    web.begin(&shared);

    Serial.println("System started");
}

void loop() {
    web.handle();
    vTaskDelay(pdMS_TO_TICKS(10));
}